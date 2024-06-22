# cs225a-baseball
This repository contains the code for the Baseball batting robot project developes for CS 225A : Experimentsal Robotics (Spring 2024). The authors who worked on this repo are Ryan Nguyen, Omar Escoto, Charles Joyner, Aby Jose. 
![image](https://github.com/aby25jose/cs225a-baseball/assets/114952711/7876b901-31f2-4ed8-b29f-ad7ffbfc86aa)

Video of demonstration:


https://github.com/aby25jose/cs225a-baseball/assets/114952711/cedd677e-0d73-4af7-9181-ca19eb4f8b73



## Dependencies
The project depends on the sai2 libraries, which can be installed here: https://github.com/manips-sai-org

## Build and make
In the main directory, create a build directory and build from that folder:
```
mkdir build
cd build
cmake .. && make -j4
```
## Run
Go to the bin folder and then to the folder of the application you want to run.
For example:
```
cd bin/pandabat
./controller-pandabat
```
Note: You need to run the simviz file in /bin/pandabat to load the simulation environment, before running the controller file. 
