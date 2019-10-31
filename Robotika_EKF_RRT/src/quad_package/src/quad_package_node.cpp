/*

Copyright by 
	Sudiro 
		[at] SudiroEEN@gmail.com
	guthub.com/sudiroeen

*/


#include "quad_package/quad_package.h"

int main(int argc, char **argv) { 
  ros::init(argc, argv, "quad_package_node");
  ros::NodeHandle nh_;

  string serverIP = "127.0.0.1";
  int serverPort = 19999;
  int clientID = simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
  Quad Q;
  return 0;
}