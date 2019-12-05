/*

Copyright by 
	Sudiro 
		[at] SudiroEEN@gmail.com
	guthub.com/sudiroeen

*/

#ifndef PIONEER_PACKAGE_H
#define PIONEER_PACKAGE_H

/*
#10 -> ,{0.26851588487625, 1.4222304821014, 0.32431721687317}
#9 -> ,{0.23931504786015, 1.4091914892197, 0.32882890105247}
*/
extern "C" {
  #include "common_include/extApi.h"
}

#include <rrt/rrt.h>

#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace cv;

// #define OBSTACLE_AVOIDANCE
#define LINGKARAN

typedef std::pair<float, float> TipLine;
typedef std::vector<std::pair<float, float> > TipLines;
typedef std::vector<std::pair<float, float> > TipLines_xy;
typedef std::vector<std::vector<std::pair<float, float> > >TipLiness;
typedef std::vector<std::vector<std::pair<float, float> > >TipLiness_xy;
typedef std::vector<TipLiness_xy> TipLines3;
typedef std::vector<TipLiness_xy> TipLines3_xy;
typedef std::pair< std::vector<TipLiness_xy>, std::vector<std::vector<size_t> > > PairLineIndex;
typedef std::vector<size_t> size_t_v;
typedef std::vector<size_t_v> size_t_vv;
typedef std::vector<Vec3f> Landmark_single;
typedef std::vector<Landmark_single > Landmark;

typedef std::vector<std::vector<Point2f> >OBS;

typedef struct LINE_data{
	Mat xi, yi;
	double xm, ym;
	double theta;
	double alpha;
} LINE_data;

typedef struct LINE_data_linear{
  Mat A, yi;
  Mat mc;
  double alpha;
} LINE_data_linear;

typedef std::vector<LINE_data> LINEv;
typedef std::vector<std::vector<LINE_data> > LINEvv;
typedef std::vector<std::vector<LINE_data_linear> > LVVLinear;

class Pioneer{
public:
  Pioneer(int clientID_, float R_, float l_, float w_);
  ~Pioneer();
  void loop();
  void stop();

private:
  // Motion tools
  void selfSetActuator();
  inline void setVL(float vleft_);
  inline void setVR(float vright_);
  inline void sleep_(int delay_);

  // callback vrep tools
  void getPos(const geometry_msgs::Vector3::ConstPtr& pos_orient_);
  void getLaserData(const sensor_msgs::LaserScanConstPtr& laser_data_);
  void getVelocity(const geometry_msgs::Vector3::ConstPtr& vel_lin_ang_);

  void getQuadPos(const geometry_msgs::Vector3::ConstPtr& quad_pos_val_);

  // common tools
  inline Point2f cvt2frameWorld(const float& cx_, const float& cy_);
  inline void cvt2frameWorld(Point2f& cx_cy_);
  inline float deg2rad(float deg_);

  // visualization tools
  void show();

  // laser data process tools
  void process_laser_data();
  PairLineIndex IterativeEndPointFit(const TipLiness& kumpul_line_xy_);
  TipLiness AdaptiveBreakPointDetector(const TipLines& str_ln_tp, const float& dalpha_);
  void sub_IterativeEndPointFit(const TipLines& partisi_m_const, TipLiness_xy& partisi_m_, vector<size_t>& ind_bp_fix_);
  Landmark getLandmarkObserved(const TipLines_xy& res_iepf_first_const, const PairLineIndex& res_iepf_, const string& type_);
  inline float prep_dist(const float& xk, const float& yk, const Vec3f& ABC_);
  inline float calc_dri_rips(float ri, float rips, float dalpha_);
  inline float Dthd(float ri, float dalpha);

  // localization tools
  void Observation();
  void ekf_localization_odom(const Landmark& obs_landmark, const Landmark_single& true_landmark_);
  void ekf_localization_velo(const Landmark& obs_landmark, const Landmark_single& true_landmark_);

private:
  // ROS tools
  ros::NodeHandle nh_;
  ros::Subscriber laser_data;
  ros::Subscriber pos_fix_sub;
  ros::Subscriber vel_lin_ang_sub;

  ros::Subscriber quad_pos_sub;

public:
  ros::Publisher state2hover_pub;
  ros::Publisher state2landing_pub;

private:
  // Communication API tools
  int clientID;
  float R, l, w;
  int leftMotorHandle;
  int rightMotorHandle;
  int pioneer_handle;
  string sensorNome[16];
  int sensorHandle[16];

  // Motion tools
  float noDetectionDist;
  float maxDetectionDist;
  float detect[16];
  float braitenbergL[16];
  float braitenbergR[16];
  float vLeft, vRight, v0;

  // visualization tools
  int FRAME_WIDTH, FRAME_HEIGHT;
  int FIELD_WIDTH, FIELD_HEIGHT;
  Mat img_bck;
  Mat img_bck_clone;
  Mat img_laser;

  // Laser tools
  std::vector<float> ranges_laser;
  int size_ranges_data;
  float angle_min_laser;
  float angle_max_laser;
  float angle_incr_laser;

  // Landmark tools
  Point2f VERTEX_4;
  Point2f VERTEX_5;
  Point2f VERTEX_6;
  Point2f VERTEX_7;

  Point2f VERTEX_11;
  Point2f VERTEX_12;
  Point2f VERTEX_13;

  // Line process tools
  TipLiness tpln_close;
  PairLineIndex res_iepf;
  float iepf_thd = 10.0;

  // Localization tools
  double DT;
  double time_before;

  Mat Omegat;
  Mat Mt;
  
  float orientation;
  float orientation_dr, orientation_dr_awal;
  float orientation_est, orientation_est_awal;

  Point2f position;
  Point2f position_dr, position_dr_awal;
  Point2f position_est, position_est_awal;

  bool isAwal;
  Point2f velocity_xy;
  float velocity_w;  
  Landmark_single true_landmark;
  Landmark observed_landmark;
  Point2f tujuan_xy;
  Point2f tujuan_xy_dr;
  Point2f tujuan_xy_est;

  bool estAwal;
  float del_x;
  float del_y;
  float drot1;
  float drot2;

  Mat Qt, I, input_loc;
  float sigma_r, sigma_phi, sigma_s;
  float dtrans, alpha_1, alpha_2, alpha_3, alpha_4;

// util for path planning
public:
  OBS _obs_;
  Point2f start_path;
  Point2f finish_path;
  PATH nodePath;
  size_t nstate, stateNow, nextState;
  Point2f TargetNow, posNow;
  bool rotate = true;

  float aim_angle;
  float etheta;

  float p_gain;
  float rotate_gain;
  bool diam;

public:
  void initializeFollowing(PATH path2follow);
  void pathFollowing();

private:
  float calc_angle(Point2f psnw, Point2f tgtnw);
  inline std::vector<Point2f> World2Frame(const vector<Point2f>& worldPos);

private:
  float orientation_plan;
  Point2f quad_position;
  PATH nodePath_in;

  float past_error;
  float error_now;
  bool first_ = true;
  bool first_v = true;
  float vpast, vnow;

  void handleRotate();

public:
  std_msgs::Bool state2hover_;
  std_msgs::Bool state2landing_;
};

#endif
