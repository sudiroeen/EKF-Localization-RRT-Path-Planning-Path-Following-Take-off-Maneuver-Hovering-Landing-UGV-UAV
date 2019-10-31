/*

Copyright by 
    Sudiro 
        [at] SudiroEEN@gmail.com
    guthub.com/sudiroeen

*/


extern "C" {
  #include "common_include/extApi.h"
}

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
  														  nav_msgs::Odometry,
  														  nav_msgs::Odometry,
  														  nav_msgs::Odometry> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class Quad{
public:	
	Quad();
	void loop();
	void fullOdomCb(const nav_msgs::OdometryConstPtr &odom_base_sub_,
	                      const nav_msgs::OdometryConstPtr &odom_targetObj_sub_,
	                      const nav_msgs::OdometryConstPtr &odom_heli_sub_,
	                      const nav_msgs::OdometryConstPtr &odom_matrix_sub_);
	void publishSpeed(float* cSpeed);
    int count;

private:
	int clientID;
    int targetObj_handle;
	
	float* targetPos;
	float* basePos;
	float* heliPos;

	float* targetOrient;
	float* baseOrient;

	float eThdTarget;
	float cumulE;
	float cumul;
    float lastE;

	float pParam;
    float iParam;
    float dParam;
    float vParam;

    float pAlphaE;
    float pBetaE;
    float TempPosTargetFromBase2;
    float TempPosTargetFromBase1;
    float prevZBaseFromTarget;

    float* propellerVelocity;
    float* matrixPos;
    float* posTargetFromBase;
    float* OrientBaseThdTarget;

    float* vx;
    float* vy;

    float* PengenPos;
    float* cusPos;
    float* posTargetNow;
    float* awalPos;
    float Dx;
    float Dy;
    float Dz;

    float stepXY;
    float stepZ;

    float dx, dxn;
    float dy, dyn;
    float dz;

    bool udah_sampai_xy;
    bool getAwalPos;

	ros::NodeHandle nh_;
	
	ros::Publisher pub_velPropeller1;
	ros::Publisher pub_velPropeller2;
	ros::Publisher pub_velPropeller3;
	ros::Publisher pub_velPropeller4;
    ros::Publisher setPos;

	message_filters::Subscriber<nav_msgs::Odometry> odom_base_sub;
	message_filters::Subscriber<nav_msgs::Odometry> odom_targetObj_sub;
	message_filters::Subscriber<nav_msgs::Odometry> odom_heli_sub;
	message_filters::Subscriber<nav_msgs::Odometry> odom_matrix_sub;

	boost::shared_ptr<Sync> sync_;
};