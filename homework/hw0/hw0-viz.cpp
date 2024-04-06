#include "simviz/SimVizRedisInterface.h"
#include "simviz/SimVizConfigParser.h"

int main(int argc, char** argv) {
	Sai2Model::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);
	Sai2Model::URDF_FOLDERS["HW0_FOLDER"] = std::string(HW_FOLDER) + "/hw0";
	std::string config_file = std::string(HW_FOLDER) + "/hw0/simviz_config.xml";
	Sai2Interfaces::SimVizConfigParser parser;
	Sai2Interfaces::SimVizRedisInterface simviz(parser.parseConfig(config_file));
	simviz.run();
	return 0;
}
