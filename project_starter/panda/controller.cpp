/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <deque>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// States 
enum State {
	POSTURE = 0, 
	MOTION
};

VectorXd savitzky_golay_coefficients(int window_size, int poly_order) {
    int half_window = (window_size - 1) / 2;
    MatrixXd A(window_size, poly_order + 1);
    VectorXd t(window_size);
    for (int i = -half_window; i <= half_window; ++i) {
        t(i + half_window) = i;
    }
    for (int i = 0; i < window_size; ++i) {
        for (int j = 0; j <= poly_order; ++j) {
            A(i, j) = pow(t(i), j);
        }
    }
    MatrixXd ATA = A.transpose() * A;
    VectorXd b = A.col(1); // Derivative corresponds to the first power term
    VectorXd coeffs = ATA.ldlt().solve(b);
    return coeffs;
}


int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_hand.urdf";

	// initial state 
	int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0);

	Vector3d ee_pos;
	Matrix3d ee_ori;

	// gripper partial joint task 
	MatrixXd gripper_selection_matrix = MatrixXd::Zero(2, robot->dof());
	gripper_selection_matrix(0, 7) = 1;
	gripper_selection_matrix(1, 8) = 1;
	auto gripper_task = std::make_shared<Sai2Primitives::JointTask>(robot, gripper_selection_matrix);
	gripper_task->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::IMPEDANCE);
	double kp_gripper = 5e3;
	double kv_gripper = 1e2;
	gripper_task->setGains(kp_gripper, kv_gripper, 0);
	gripper_task->setGains(kp_gripper, kv_gripper, 0);

	// joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	VectorXd q_desired(dof);
	q_desired.head(7) << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_desired.head(7) *= M_PI / 180.0;
	q_desired.tail(2) << 0.04, -0.04;
	joint_task->setGoalPosition(q_desired);

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	double data_freq = 120;       // Position data frequency in Hz
	const int window_size = 5;   // Choose an odd number
	const int poly_order = 2;
	Sai2Common::LoopTimer timer(control_freq, 1e6);
	VectorXd sg_coeffs = savitzky_golay_coefficients(window_size, poly_order);
    deque<Vector3d> position_history;
	deque<double> time_history;

	long long int data_counter = 0;
    int data_interval = static_cast<int>(control_freq / data_freq);

	// Step 1: Calibration
	Vector3d OS_X_0 = robot->position(control_link, control_point);
	Matrix3d OS_R_0 = robot->rotation(control_link);

	Matrix3d OS_R_OT;
	OS_R_OT << -1, 0, 0,
				0, 0, 1,
				0, 1, 0;
	OS_R_OT = AngleAxisd(M_PI, Vector3d (0,0,1)).toRotationMatrix() * OS_R_OT;
	
	Vector3d OT_X_0 = redis_client.getEigen(RIGID_BODY_POS);
	Vector4d OT_Q_0 = redis_client.getEigen(RIGID_BODY_ORI);
	Quaterniond quaternion(OT_Q_0[3], OT_Q_0[0], OT_Q_0[1], OT_Q_0[2]);
	Matrix3d OT_R_0 = quaternion.toRotationMatrix(); 	// Convert the quaternion to a rotation matrix


	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		Vector3d OS_Xd;
		Matrix3d OS_Rd;


		if (data_counter % data_interval == 0 ) {

			// Step 2: relative position and orientation read 
			Vector3d pos = redis_client.getEigen(RIGID_BODY_POS);
			Vector3d OT_X = pos;

			Vector4d quat = redis_client.getEigen(RIGID_BODY_ORI);
			Quaterniond quatObj(quat[3], quat[0], quat[1], quat[2]);
			Matrix3d OT_R = quatObj.toRotationMatrix();

			Vector3d OT_X_rel = OT_X - OT_X_0;
			Matrix3d OT_R_rel = OT_R.transpose() * OT_R_0;

			// Step 3: tracking position and orientation in OpenSai Frame
			Vector3d OS_X = OS_X_0 + OS_R_OT * OT_X_rel;
			Matrix3d OS_R = OS_R_OT * OT_R_rel * OS_R_OT.transpose() * OS_R_0;

			OS_Xd = OS_X;
			OS_Rd = OS_R;

			//debugging prints
			cout << OS_X << endl;

			// Add position and time to history
			position_history.push_back(OS_X);
			time_history.push_back(time);

			// Ensure we have enough data points to compute the velocity
			if (position_history.size() > 1) {
				// Compute the finite difference for velocity
				Vector3d pos_prev = position_history[position_history.size() - 2];
				double time_prev = time_history[time_history.size() - 2];

				double dt = time - time_prev;
				
				Vector3d velocity = (OS_X - pos_prev) / dt;

				// Print the smoothed velocity
				cout << "Time: " << time << " Velocity: " << velocity.transpose() << endl;

			// Keep only the latest 2 positions and times
			if (position_history.size() > 2) {
				position_history.pop_front();
				time_history.pop_front();
			}
        }

		}

        data_counter++;
	
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			command_torques = joint_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				gripper_task->reInitializeTask();
				joint_task->reInitializeTask();

				ee_pos = robot->position(control_link, control_point);
				ee_ori = robot->rotation(control_link);

				// pose_task->setGoalPosition(ee_pos - Vector3d(-0.1, -0.1, 0.1));
				// pose_task->setGoalOrientation(AngleAxisd(M_PI / 6, Vector3d::UnitX()).toRotationMatrix() * ee_ori);
				gripper_task->setGoalPosition(Vector2d(0.02, -0.02));

				state = MOTION;
			}
		} else if (state == MOTION) {
			// update goal position and orientation
			pose_task->setGoalPosition(OS_Xd);
			pose_task->setGoalOrientation(OS_Rd);

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			gripper_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			joint_task->updateTaskModel(gripper_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + gripper_task->computeTorques() + joint_task->computeTorques();
		}

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
