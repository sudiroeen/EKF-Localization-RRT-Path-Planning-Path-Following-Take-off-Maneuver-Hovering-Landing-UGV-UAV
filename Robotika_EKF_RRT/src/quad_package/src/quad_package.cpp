/*

Copyright by 
  Sudiro 
    [at] SudiroEEN@gmail.com
  guthub.com/sudiroeen

*/


#include "quad_package/quad_package.h"

Quad::Quad()
     :nh_(ros::this_node::getName()),
      pub_velPropeller1(nh_.advertise<std_msgs::Float32>("/velPropeller1", 1)),
      pub_velPropeller2(nh_.advertise<std_msgs::Float32>("/velPropeller2", 1)),
      pub_velPropeller3(nh_.advertise<std_msgs::Float32>("/velPropeller3", 1)),
      pub_velPropeller4(nh_.advertise<std_msgs::Float32>("/velPropeller4", 1)),
      setPos(nh_.advertise<geometry_msgs::Vector3>("/set_cuPos", 1)),
      odom_base_sub(nh_, "/odom_base",1),
      odom_targetObj_sub(nh_, "/odom_targetObj",1),
      odom_heli_sub(nh_, "/odom_heli",1),
      odom_matrix_sub(nh_, "/odom_matrix",1),
      targetPos(new float[3]), basePos(new float[3]), heliPos(new float[3]), matrixPos(new float[12]),
      targetOrient(new float[3]), baseOrient(new float[3]), OrientBaseThdTarget(new float[3]),
      eThdTarget(0.0), cumulE(0.0), lastE(0.0), pAlphaE(0.0), pBetaE(0.0),
      pParam(2.0), iParam(0.0), dParam(0.0), vParam(-2.0), cusPos(new float[3]),
      posTargetFromBase(new float[3]), TempPosTargetFromBase1(0.0), TempPosTargetFromBase2(0.0), prevZBaseFromTarget(0.0),
      vx(new float[3]), vy(new float[3]), getAwalPos(true), udah_sampai_xy(false), propellerVelocity(new float[4]),
      stepXY(300.0), stepZ(100), posTargetNow(new float[3]), PengenPos(new float[3]), awalPos(new float[3]), count(0)
{
	PengenPos[0] = 0.0;
	PengenPos[1] = 0.0;
	PengenPos[2] = 0.6;
    
    sync_.reset(new Sync(MySyncPolicy(10), odom_base_sub, odom_targetObj_sub, odom_heli_sub, odom_matrix_sub));
    sync_->registerCallback(boost::bind(&Quad::fullOdomCb, this, _1, _2, _3, _4));

    ros::Rate loop_rate(10);
	while(ros::ok()){
	  
	  loop_rate.sleep();
	  ros::spinOnce();
	}
}

void Quad::loop(){
  eThdTarget = targetPos[2] - basePos[2];
  cumulE += eThdTarget;
  float proportional_value = pParam * eThdTarget;
  float thrust = 8.0 + proportional_value + iParam * cumulE + dParam * (eThdTarget - lastE) + heliPos[2] * vParam;
  lastE = eThdTarget;

  if((fabs(eThdTarget) < 0.01) && udah_sampai_xy){
  	cusPos[2] = cusPos[2] + dz;

    geometry_msgs::Vector3 cusPos_msgs;
  	cusPos_msgs.x = cusPos[0];
  	cusPos_msgs.y = cusPos[1];
  	cusPos_msgs.z = cusPos[2]; 
  	setPos.publish(cusPos_msgs);
  }

  for(int i=0; i<3; i++){
    posTargetFromBase[i] = targetPos[i] - basePos[i];
  }

  float alphaE = vy[2] - matrixPos[11];
  float alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - pAlphaE);
  pAlphaE = alphaE;

  float betaE = vx[2] - matrixPos[11];
  float betaCorr = -0.25 * betaE - 2.1 * (betaE - pBetaE);
  pBetaE = betaE;

  alphaCorr = alphaCorr + posTargetFromBase[1] * 0.005 + 1 * (posTargetFromBase[1] - TempPosTargetFromBase2);
  betaCorr = betaCorr - posTargetFromBase[0] * 0.005 - 1 * (posTargetFromBase[0] - TempPosTargetFromBase1);
  TempPosTargetFromBase2 = posTargetFromBase[1];
  TempPosTargetFromBase1 = posTargetFromBase[0];

  dxn = targetPos[0] - PengenPos[0];
  dyn = targetPos[1] - PengenPos[1];

  if(sqrt(pow(dxn,2) + pow(dyn, 2)) <= 0.01){
  	udah_sampai_xy = true;
  }

  if((sqrt(pow(alphaE, 2) + pow(betaE, 2)) <= 0.01) && !udah_sampai_xy){
  	   cusPos[0] = cusPos[0] + dx;
       cusPos[1] = cusPos[1] + dy;

       geometry_msgs::Vector3 cusPos_msgs;
       cusPos_msgs.x = cusPos[0];
       cusPos_msgs.y = cusPos[1];
       cusPos_msgs.z = cusPos[2]; 
       setPos.publish(cusPos_msgs);
  }

  for(int i=0; i<3; i++){
    OrientBaseThdTarget[i] = baseOrient[i] - targetOrient[i];
  }
  float rotCorr = OrientBaseThdTarget[2] * 0.1 + 2 * (OrientBaseThdTarget[2] - prevZBaseFromTarget);
  prevZBaseFromTarget = OrientBaseThdTarget[2];

  propellerVelocity[0] = thrust * (1 - alphaCorr + betaCorr + rotCorr);
  propellerVelocity[1] = thrust * (1 - alphaCorr - betaCorr - rotCorr);
  propellerVelocity[2] = thrust * (1 + alphaCorr - betaCorr + rotCorr);
  propellerVelocity[3] = thrust * (1 + alphaCorr + betaCorr - rotCorr);

  publishSpeed(propellerVelocity);	
}

void Quad::publishSpeed(float* cSpeed){
  std_msgs::Float32* cs_msg = new std_msgs::Float32[4];
  for(int i=0; i<4; i++){
     cs_msg[i].data = cSpeed[i];
  }
  pub_velPropeller1.publish(cs_msg[0]);
  pub_velPropeller2.publish(cs_msg[1]);
  pub_velPropeller3.publish(cs_msg[2]);
  pub_velPropeller4.publish(cs_msg[3]);
}


void Quad::fullOdomCb(const nav_msgs::OdometryConstPtr &odom_base_sub_,
                      const nav_msgs::OdometryConstPtr &odom_targetObj_sub_,
                      const nav_msgs::OdometryConstPtr &odom_heli_sub_,
                      const nav_msgs::OdometryConstPtr &odom_matrix_sub_)
{
  // count ++;
  basePos[0] = odom_base_sub_->pose.pose.position.x;
  basePos[1] = odom_base_sub_->pose.pose.position.y;
  basePos[2] = odom_base_sub_->pose.pose.position.z;

  baseOrient[0] = odom_base_sub_->pose.pose.orientation.x;
  baseOrient[1] = odom_base_sub_->pose.pose.orientation.y;
  baseOrient[2] = odom_base_sub_->pose.pose.orientation.z;

  if(getAwalPos){
  	Dx = PengenPos[0] - basePos[0];
    Dy = PengenPos[1] - basePos[1];
    Dz = PengenPos[2] - basePos[2];

    stepXY = 1000.0;
    stepZ = 500.0;
    
    dx = Dx/stepXY;
    dy = Dy/stepXY;
    dz = Dz/stepZ;

    for(int i=0; i<3; i++)
    	cusPos[i] = basePos[i];

    getAwalPos = false;
  }

  targetPos[0] = odom_targetObj_sub_->pose.pose.position.x;
  targetPos[1] = odom_targetObj_sub_->pose.pose.position.y;
  targetPos[2] = odom_targetObj_sub_->pose.pose.position.z;

  targetOrient[0] = odom_targetObj_sub_->pose.pose.orientation.x;
  targetOrient[1] = odom_targetObj_sub_->pose.pose.orientation.y;
  targetOrient[2] = odom_targetObj_sub_->pose.pose.orientation.z;

  heliPos[0] = odom_heli_sub_->twist.twist.linear.x;
  heliPos[1] = odom_heli_sub_->twist.twist.linear.y;
  heliPos[2] = odom_heli_sub_->twist.twist.linear.z;

  matrixPos[0] = odom_matrix_sub_->pose.pose.position.x;
  matrixPos[1] = odom_matrix_sub_->pose.pose.position.y;
  matrixPos[2] = odom_matrix_sub_->pose.pose.position.z;

  matrixPos[3] = odom_matrix_sub_->pose.pose.orientation.x;
  matrixPos[4] = odom_matrix_sub_->pose.pose.orientation.y;
  matrixPos[5] = odom_matrix_sub_->pose.pose.orientation.z;

  matrixPos[6] = odom_matrix_sub_->twist.twist.linear.x;
  matrixPos[7] = odom_matrix_sub_->twist.twist.linear.y;
  matrixPos[8] = odom_matrix_sub_->twist.twist.linear.z;

  matrixPos[9] = odom_matrix_sub_->twist.twist.angular.x;
  matrixPos[10] = odom_matrix_sub_->twist.twist.angular.y;
  matrixPos[11] = odom_matrix_sub_->twist.twist.angular.z;

  vx[0] = matrixPos[0] + matrixPos[3];
  vx[1] = matrixPos[4] + matrixPos[7];
  vx[2] = matrixPos[8] + matrixPos[11];

  vy[0] = matrixPos[1] + matrixPos[3];
  vy[1] = matrixPos[5] + matrixPos[7];
  vy[2] = matrixPos[9] + matrixPos[11];

  loop();
}