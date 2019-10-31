/*

Copyright by 
  Sudiro 
    [at] SudiroEEN@gmail.com
  guthub.com/sudiroeen

*/


#include "pioneer_package/pioneer_package.h"

int main(int argc, char **argv) { 
  ros::init(argc, argv, "pioneer_package_node");
  string serverIP = "127.0.0.1";
  int serverPort = 19999;
  int clientID = simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
  
  float lebar_robot = 0.331;
  float jari_jari_putar = 1.75;
  float kecepatan_sudut = 2.0;
  Pioneer P(clientID, jari_jari_putar, lebar_robot, kecepatan_sudut);

  ros::Rate loop_rate(10);

  if(clientID != -1){
  	while(simxGetConnectionId(clientID)!=-1){
  	  P.loop();
  	  ros::spinOnce();
  	  loop_rate.sleep();
      int k = waitKey(1);
      if((char)k == 'p'){
        P.state2hover_.data = true;
        int i = 0;
        while(i < 50){
          P.stop();
          P.state2hover_pub.publish(P.state2hover_);
          i++;
          ros::spinOnce();
          loop_rate.sleep();
          waitKey(1);
        }
        break;
      }
  	}

    P.stop();
    destroyAllWindows();

    PathPlanning PPrrt(P.start_path, P.finish_path, P._obs_);
    PPrrt.execute();
    P.initializeFollowing(PPrrt.getPathRRT);

    while(simxGetConnectionId(clientID)!=-1){
      P.pathFollowing();
      ros::spinOnce();
      loop_rate.sleep();
      waitKey(1);
    }
  	simxFinish(clientID);
  }
  return 0;
}