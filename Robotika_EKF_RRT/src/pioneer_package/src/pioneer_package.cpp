/*

Copyright by 
	Sudiro 
		[at] SudiroEEN@gmail.com
	guthub.com/sudiroeen

*/


#include "pioneer_package/pioneer_package.h"

inline float Pioneer::deg2rad(float deg_){
  return deg_ * M_PI/180.0;
}

inline void Pioneer::setVL(float vleft_){
    simxSetJointTargetVelocity(clientID, leftMotorHandle, (simxFloat) vleft_, simx_opmode_streaming);
}

inline void Pioneer::setVR(float vright_){
    simxSetJointTargetVelocity(clientID, rightMotorHandle, (simxFloat) vright_, simx_opmode_streaming);
}

inline void Pioneer::sleep_(int delay_){
  extApi_sleepMs(delay_);
}

inline void Pioneer::cvt2frameWorld(Point2f& cx_cy_){
  cx_cy_ = cvt2frameWorld(cx_cy_.x, cx_cy_.y);
}

inline float Pioneer::Dthd(float ri, float dalpha){
	float lambda_ = 10.0; // degree
	float dr = 0.01; // meter
	float dthd_ = ri* sin(dalpha)/(sin(deg2rad(lambda_) - dalpha)) + 3*dr;
	return fabs(dthd_);
}

inline float Pioneer::calc_dri_rips(float ri, float rips, float dalpha_){
	return sqrt(ri*ri + rips*rips - 2*ri*rips*cos(dalpha_));
}

inline Point2f Pioneer::cvt2frameWorld(const float& cx_, const float& cy_){
	Point2f point_val_;
	point_val_.x = -float(FRAME_WIDTH) / float(FIELD_WIDTH) * cx_ + float(FRAME_WIDTH)/2.0;
	point_val_.y = float(FRAME_HEIGHT) / float(FIELD_HEIGHT) * cy_ + float(FRAME_HEIGHT)/2.0;
	return point_val_;
}

inline float Pioneer::prep_dist(const float& xk, const float& yk, const Vec3f& ABC_){
	return fabs(ABC_[0]*xk + ABC_[1]*yk + ABC_[2])/sqrt(pow(ABC_[0], 2) + pow(ABC_[1], 2));
}

inline std::vector<Point2f> Pioneer::World2Frame(const vector<Point2f>& worldPos){
	std::vector<Point2f> vpos= worldPos;
	for(size_t i=0; i<worldPos.size(); i++){
		vpos[i].x = -float(FIELD_WIDTH) / float(FRAME_WIDTH) * float(worldPos[i].x - FRAME_WIDTH/2.0);
		vpos[i].y = float(FIELD_HEIGHT) / float(FRAME_HEIGHT) * float(worldPos[i].y - FRAME_HEIGHT/2.0);
	}
	return vpos;
}

void Pioneer::getQuadPos(const geometry_msgs::Vector3::ConstPtr& quad_pos_val_){
	quad_position.x = quad_pos_val_->x;
	quad_position.y = quad_pos_val_->y;
}

void Pioneer::stop(){
	setVL(0.0);
	setVR(0.0);
}

Pioneer::~Pioneer(){
	destroyAllWindows();
}

Pioneer::Pioneer(int clientID_, float R_, float l_, float w_)
        : clientID(clientID_),
          leftMotorHandle(-1),
          rightMotorHandle(-1),
          R(R_), l(l_), w(w_),
          VERTEX_4(Point2f(-2.4258, -2.4801)), // Nomor 4 pada blk di file .ttt
          VERTEX_5(Point2f(-3.2258, 2.1199)), // Nomor 5 pada blk di file .ttt
          VERTEX_6(Point2f(4.2492, 2.1205)), // Nomor 6 pada blk di file .ttt
          VERTEX_7(Point2f(2.5492, -2.4851)), // Nomor 7 pada blk di file .ttt
          VERTEX_11(Point2f(1.6250e+00, -7.2497e-01)),
          VERTEX_13(Point2f(5.2505e-01, -5.2497e-01)),
          VERTEX_12(Point2f(1.2505e-01, 2.7503e-01)),
          nh_(ros::this_node::getName()),
          laser_data(nh_.subscribe<sensor_msgs::LaserScan>("/laser_data", 1, &Pioneer::getLaserData, this)),
          pos_fix_sub(nh_.subscribe<geometry_msgs::Vector3>("/pos_orient_", 1, &Pioneer::getPos, this)),
          vel_lin_ang_sub(nh_.subscribe<geometry_msgs::Vector3>("/vel_lin_ang", 1, &Pioneer::getVelocity, this)),
          quad_pos_sub(nh_.subscribe<geometry_msgs::Vector3>("/quad_pos", 1, &Pioneer::getQuadPos, this)),
          state2hover_pub(nh_.advertise<std_msgs::Bool>("/state2hover_topic", 1)),
          state2landing_pub(nh_.advertise<std_msgs::Bool>("/state2landing_topic", 1)),
          FRAME_WIDTH(600), FRAME_HEIGHT(600), FIELD_WIDTH(15), FIELD_HEIGHT(15), v0(2.0),
          braitenbergL({-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}),
          braitenbergR({-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}), detect({0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}),
          maxDetectionDist(0.2), noDetectionDist(0.5), sigma_r(0.1), sigma_phi(deg2rad(5.0)), sigma_s(deg2rad(5.0)),
          alpha_1(0.1), alpha_2(0.1), alpha_3(0.1), alpha_4(0.1)
{
    cvt2frameWorld(VERTEX_4); cvt2frameWorld(VERTEX_5); cvt2frameWorld(VERTEX_6); cvt2frameWorld(VERTEX_7);
    cvt2frameWorld(VERTEX_11); cvt2frameWorld(VERTEX_12); cvt2frameWorld(VERTEX_13);

    Point2f center_rotate = Point2f(0.75, -0.25);
    cvt2frameWorld(center_rotate);
    
    cout << "center_rotate: " << center_rotate << endl
    	 << "VERTEX_4: " << VERTEX_4 << endl;

    true_landmark.resize(4);
    true_landmark.emplace_back(Vec3f(VERTEX_4.x, VERTEX_4.y, deg2rad(100.0)));
    true_landmark.emplace_back(Vec3f(VERTEX_5.x, VERTEX_5.y, deg2rad(80.0)));
    true_landmark.emplace_back(Vec3f(VERTEX_6.x, VERTEX_6.y, deg2rad(70.0)));
    true_landmark.emplace_back(Vec3f(VERTEX_7.x, VERTEX_7.y, deg2rad(110.0)));

    simxGetObjectHandle(clientID,(const simxChar*) "Pioneer_p3dx_leftMotor#0",(simxInt *) &leftMotorHandle, (simxInt) simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID,(const simxChar*) "Pioneer_p3dx_rightMotor#0",(simxInt *) &rightMotorHandle, (simxInt) simx_opmode_oneshot_wait);

    for(int i = 0; i < 16; i++){
        sensorNome[i] = "Pioneer_p3dx_ultrasonicSensor" + to_string(i + 1) + "#0";
        simxGetObjectHandle(clientID,(const simxChar*) sensorNome[i].c_str(),(simxInt *) &sensorHandle[i], (simxInt) simx_opmode_oneshot_wait);
        simxReadProximitySensor(clientID,sensorHandle[i],NULL,NULL,NULL,NULL,simx_opmode_streaming);
    }

    img_bck = Mat(FRAME_WIDTH, FRAME_HEIGHT, CV_8UC3, Scalar::all(255));
    line(img_bck, VERTEX_4, VERTEX_5, Scalar::all(0), 5);
    line(img_bck, VERTEX_5, VERTEX_6, Scalar::all(0), 5);
    line(img_bck, VERTEX_6, VERTEX_7, Scalar::all(0), 5);
    line(img_bck, VERTEX_7, VERTEX_4, Scalar::all(0), 5);

    line(img_bck, VERTEX_11, VERTEX_13, Scalar::all(0), 8);
    line(img_bck, VERTEX_13, VERTEX_12, Scalar::all(0), 8);

    circle(img_bck, center_rotate, 5, Scalar(0,0,255), -1);

	float ujung_true_x = 10.0 + 15.0;
	float ujung_true_y = 10.0;
	circle(img_bck, Point2f(10.0, 10.0), 5, Scalar(0,255,0), -1);
	line(img_bck, Point2f(10.0, 10.0), Point2f(ujung_true_x, ujung_true_y), Scalar(255,255,0), 2);
	putText(img_bck, "VREP True Position", Point(40,10), FONT_HERSHEY_DUPLEX, 0.3, Scalar(0,143,143), 1);

	float ujung_dr_x = 10.0 + 15.0;
	float ujung_dr_y = 30.0;
	circle(img_bck, Point2f(10.0, 30.0), 5, Scalar(0,0,255), -1);
	line(img_bck, Point2f(10.0, 30.0), Point2f(ujung_dr_x, ujung_dr_y), Scalar(255,0,0), 2);
	putText(img_bck, "Dead Reckoning", Point(40,30), FONT_HERSHEY_DUPLEX, 0.3, Scalar(0,143,143), 1);

	float ujung_est_x = 10.0 + 15.0;
	float ujung_est_y = 50.0;
	circle(img_bck, Point2f(10.0, 50.0), 5, Scalar(255,0,255), -1);
	line(img_bck, Point2f(10.0, 50.0), Point2f(ujung_est_x, ujung_est_y), Scalar(0,0,0), 2);
    putText(img_bck, "EKF estimation", Point(40,50), FONT_HERSHEY_DUPLEX, 0.3, Scalar(0,143,143), 1);

    img_bck_clone = img_bck.clone();

    _obs_.resize(2);
    _obs_[0].emplace_back(VERTEX_4);
    _obs_[0].emplace_back(VERTEX_5);
    _obs_[0].emplace_back(VERTEX_6);
    _obs_[0].emplace_back(VERTEX_7);

    Mat p = Mat(_obs_[0]);
    cout << "p: " << p << endl;

    _obs_[1].emplace_back(VERTEX_11);
    _obs_[1].emplace_back(VERTEX_13);
    _obs_[1].emplace_back(VERTEX_12);

    isAwal = true;
    estAwal = true;
    time_before = ros::Time::now().toSec();

    Omegat = (Mat_<float>(3,3) << 0.01, 0.0, 0.0,
    							  0.0, 0.01, 0.0,
    							  0.0, 0.0, deg2rad(5));

    Qt  = (Mat_<float>(3,3) << pow(sigma_r, 2), 0.0, 0.0,
    	  					   0.0, pow(sigma_phi, 2), 0.0,
    	  					   0.0, 0.0, pow(sigma_s, 2));
    I = Mat::eye(3,3, CV_32FC1);
    input_loc = Mat::zeros(2,1, CV_32FC1);
}

void Pioneer::selfSetActuator(){
#ifdef LINGKARAN
    vLeft = w*(R - l/2.0);
    vRight = w*(R + l/2.0);
#elif defined OBSTACLE_AVOIDANCE    
    for(int i = 0; i < 16; i++){
        simxUChar state;
        simxFloat coord[3];
        if (simxReadProximitySensor(clientID,sensorHandle[i],&state,coord,NULL,NULL,simx_opmode_buffer)==simx_return_ok){ 
           float dist = coord[2];
           if(state > 0 && (dist<noDetectionDist)){
              if(dist<maxDetectionDist)
              	dist=maxDetectionDist;
              detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
            }else
             detect[i] = 0;
        }else
           detect[i] = 0;
    }
    vLeft = v0;
    vRight = v0;

    for(int i = 0; i < 16; i++){
      vLeft=vLeft+braitenbergL[i]*detect[i];
      vRight=vRight+braitenbergR[i]*detect[i];
    }
#endif
    setVL(vLeft);
    setVR(vRight);
}

float Pioneer::calc_angle(Point2f psnw, Point2f tgtnw){
  return atan2(tgtnw.y - psnw.y, tgtnw.x - psnw.x);
}

void Pioneer::initializeFollowing(PATH path2follow){
  path2follow.erase(path2follow.begin());
  nodePath_in = path2follow;
  nodePath = World2Frame(path2follow);
  nstate = path2follow.size();
  stateNow = 0;
  nextState = 1;
  TargetNow = nodePath[0];
}

void Pioneer::pathFollowing(){
  float aim_angle = calc_angle(posNow, TargetNow);
  float etheta = aim_angle - orientation_plan;

  float p_gain = 0.9;
  float rotate_gain = 0.7;
  state2landing_.data = false;

  float ex = fabs(TargetNow.x - posNow.x);
  float ey = fabs(TargetNow.y - posNow.y);
  float er = ex + ey;

  bool diam = false;

  if(first_){
  	error_now = er;
  	past_error = er;
  	first_ = false;
  }

  error_now = er;

  // bagina ex dan ey nggak boleh digabung ke er untuk kasus memperhatikan tanda karena bisa aja ex + dan ey - dan bisa aja saling menghilangkan
  Point2f pnw = posNow; cvt2frameWorld(pnw);
  Point2f tgw = TargetNow; cvt2frameWorld(tgw);

  cout << "path: " << endl;
  for(size_t q=0; q<nodePath_in.size(); q++)
    cout << nodePath_in[q] << endl;
  cout << "posNow: " << pnw << "\t TargetNow: " << tgw << endl;

  if(rotate){
    if(fabs(er) < 0.8){
		cout << "next" << endl;
		if(stateNow < nstate-1){
	        stateNow = stateNow + 1;
	        rotate = false;
	    }else{
	        vRight = vLeft = 0.0;
	        state2landing_.data = true;
	        rotate = true;
	    }

		TargetNow = nodePath[stateNow];
    }else{
    	if(!first_ && (error_now < past_error)){
			vRight = vLeft = p_gain*er;
    	}else{
    		vRight = vLeft = p_gain*er;
			// vRight = vLeft = -vpast;
    	}

    	vnow = vRight;
		cout << "maju\t vLeft: " << vLeft << "\t vRight: " << vRight << endl;
    }
  }else{
    if(fabs(etheta) > 1 * M_PI/180.0){
    	if(fabs(etheta) > M_PI){
    		// cout << "etheta: " << etheta * 180.0/M_PI << "\t";
    		etheta = -(2*M_PI - etheta);
    		if(etheta < -2*M_PI)
    			etheta = etheta + 4*M_PI;
    		// cout << "etheta: " << etheta * 180.0/M_PI << endl; //"\ter: " << er << endl;
    	}
		if(etheta < 0.0){
			vLeft = rotate_gain*etheta; 
			vRight = -vLeft;
		}else if(etheta > 0.0){
			vLeft = -rotate_gain*etheta; 
			vRight = -vLeft;
		}
    }else{
		rotate = true;
		vLeft = vRight = 0.0;
		diam = true;
    }
  }

  cout << "etheta: " << etheta * 180.0/M_PI << "\ter: " << er << endl;

  if(first_v && rotate){
  	vpast = vRight;
  	vnow = vRight;
  	first_v = false;
  }

  if(diam){
  	int ab=0;
  	while(ab < 10){
  	  ab++;
  	  // cout << ab << endl;

	  setVL(vLeft);
	  setVR(vRight);

	  state2landing_pub.publish(state2landing_);
	  past_error = error_now;
	  vpast = vnow;
	}
  }else{
	setVL(vLeft);
	setVR(vRight);

	state2landing_pub.publish(state2landing_);
	past_error = error_now;
	vpast = vnow;
  }
}

void Pioneer::getLaserData(const sensor_msgs::LaserScan::ConstPtr& laser_data_){
  size_ranges_data = sizeof(laser_data_->ranges)/sizeof(laser_data_->ranges[0]);
  ranges_laser = laser_data_->ranges;
  angle_min_laser = laser_data_->angle_min;
  angle_max_laser = laser_data_->angle_max;
  angle_incr_laser = laser_data_->angle_increment;
}

/************************************** PROSES DATA LASER *****************************************/

void Pioneer::process_laser_data(){ // MAIN
	TipLines store_line_tip;
	for(size_t i=0; i<ranges_laser.size(); i++){  
	  TipLine line_tip;
	  line_tip.first = float(FRAME_WIDTH / FIELD_WIDTH)*ranges_laser[i];
	  line_tip.second = deg2rad((ranges_laser.size()*0.5 - i)*angle_incr_laser);
	  store_line_tip.emplace_back(line_tip);

	  Point2f x_y_;
	  x_y_.x = position_dr.x + line_tip.first*cos(orientation_dr + line_tip.second);
	  x_y_.y = position_dr.y + line_tip.first*sin(orientation_dr + line_tip.second);
	}
	  
	tpln_close = AdaptiveBreakPointDetector(store_line_tip, -angle_incr_laser);
	if(tpln_close.size() < 1)
	  return;

	TipLiness_xy kumpul_line_xy;
	kumpul_line_xy.resize(tpln_close.size());
	for(size_t j=0; j<tpln_close.size(); j++){
	   kumpul_line_xy[j].resize(tpln_close[j].size());
	   for(size_t k=0; k<tpln_close[j].size(); k++){
	      Point2f x_y_;
	  	  x_y_.x = position_dr.x + tpln_close[j][k].first*cos(orientation_dr + tpln_close[j][k].second);
	  	  x_y_.y = position_dr.y + tpln_close[j][k].first*sin(orientation_dr + tpln_close[j][k].second);
	  	  kumpul_line_xy[j][k].first = x_y_.x;
	  	  kumpul_line_xy[j][k].second = x_y_.y;
	   }
	}
	res_iepf = IterativeEndPointFit(kumpul_line_xy);
	observed_landmark = getLandmarkObserved(kumpul_line_xy[0], res_iepf, "biasa");
}

TipLiness Pioneer::AdaptiveBreakPointDetector(const TipLines& str_ln_tp, const float& dalpha_){
	TipLiness kumpulan_tiplines;
	TipLines temp_kumpulan_tiplines;

	if(str_ln_tp.size() <2){
		return kumpulan_tiplines;
	}

	for(size_t i=0; i< str_ln_tp.size()-1; i++){
		if(calc_dri_rips(str_ln_tp[i].first, str_ln_tp[i+1].first, dalpha_) <= Dthd(str_ln_tp[i].first, dalpha_)){
			temp_kumpulan_tiplines.emplace_back(str_ln_tp[i]);
		}else{
			kumpulan_tiplines.emplace_back(temp_kumpulan_tiplines);
			temp_kumpulan_tiplines.clear();
		}
	}
	kumpulan_tiplines.emplace_back(temp_kumpulan_tiplines);
	return kumpulan_tiplines;
}

void Pioneer::sub_IterativeEndPointFit(const TipLines& partisi_m_const, TipLiness_xy& partisi_m_, vector<size_t>& ind_bp_fix_){
	bool bool_return = false;
	
	TipLiness_xy partisi_m_fix;
	for(size_t spm=0; spm<partisi_m_.size(); spm++){
		TipLines kumpul_line_xy_m = partisi_m_[spm];
		size_t size_kl_m = kumpul_line_xy_m.size();

		if(size_kl_m <1){
			if(spm == (partisi_m_.size() - 1))
				return;
			else
				continue;
		}
		Point2f init_start;
		Point2f init_final;
		Vec3f ABC;
		float max_dist_prep = 0.0;
		size_t ind_bp = 0;

		init_start = Point2f(kumpul_line_xy_m[0].first, kumpul_line_xy_m[0].second);
		init_final = Point2f(kumpul_line_xy_m[size_kl_m - 1].first, kumpul_line_xy_m[size_kl_m - 1].second);

		ABC[0] = init_final.y - init_start.y;
		ABC[1] = init_start.x - init_final.x;
		ABC[2] = -ABC[0]*init_start.x - ABC[1]*init_start.y;
		
		for(size_t n=0; n<size_kl_m; n++){
			circle(img_laser, Point2f(kumpul_line_xy_m[n].first, kumpul_line_xy_m[n].second), 2, Scalar(spm*150, 255-spm*130, spm*60), -1);
			float dist_prep = prep_dist(kumpul_line_xy_m[n].first, kumpul_line_xy_m[n].second, ABC);
			if(dist_prep > iepf_thd){
				if(dist_prep > max_dist_prep){
					max_dist_prep = dist_prep;
					if(spm>0)
						ind_bp = spm*partisi_m_[spm-1].size() + n;
					else
						ind_bp = n;
				}
			}
		}
		
		if(std::find(ind_bp_fix_.begin(), ind_bp_fix_.end(), ind_bp) == ind_bp_fix_.end()){
			ind_bp_fix_.insert(ind_bp_fix_.end()-1, ind_bp);
		}else if(spm == (partisi_m_.size() - 1)){
			bool_return = true;
		}
	}
	circle(img_laser, Point2f(partisi_m_const[ind_bp_fix_[0]].first, partisi_m_const[ind_bp_fix_[0]].second), 5, Scalar(0,0,255), -1);
	for(size_t r=1; r< ind_bp_fix_.size(); r++){
		TipLines temp_bps;
		circle(img_laser, Point2f(partisi_m_const[ind_bp_fix_[r]].first, partisi_m_const[ind_bp_fix_[r]].second), 5, Scalar::all(125), -1);
		if(ind_bp_fix_[r-1] <= ind_bp_fix_[r]){
			for(size_t c=ind_bp_fix_[r-1]; c<= ind_bp_fix_[r]; c++){
				temp_bps.emplace_back(partisi_m_const[c]);
			}
		}else{
			for(size_t c=ind_bp_fix_[r-1]; c> ind_bp_fix_[r]; c--){
				temp_bps.emplace_back(partisi_m_const[c]);
			}
		}
		partisi_m_fix.emplace_back(temp_bps);
	}

	partisi_m_ =  partisi_m_fix;

	if(bool_return)
		return;
	sub_IterativeEndPointFit(partisi_m_const, partisi_m_, ind_bp_fix_);
}

PairLineIndex Pioneer::IterativeEndPointFit(const TipLiness& kumpul_line_xy_){
	std::vector<TipLiness_xy> partisi_;
	std::vector<std::vector<size_t> > v_ind_bp_;

	for(size_t m=0; m<kumpul_line_xy_.size(); m++){
		TipLiness_xy partisi_m;
		partisi_m.emplace_back(kumpul_line_xy_[m]);
		TipLines partisi_m_const_ = kumpul_line_xy_[m];

		vector<size_t> ind_bp_;
		ind_bp_.emplace_back(0);
		ind_bp_.emplace_back(kumpul_line_xy_[m].size()-1);

		sub_IterativeEndPointFit(partisi_m_const_, partisi_m, ind_bp_);
		partisi_.emplace_back(partisi_m);
		v_ind_bp_.emplace_back(ind_bp_);
	}

	PairLineIndex rPairLineIndex;
	rPairLineIndex.first = partisi_;
	rPairLineIndex.second = v_ind_bp_;
	return rPairLineIndex;
}

Landmark Pioneer::getLandmarkObserved(const TipLines_xy& res_iepf_first_const, const PairLineIndex& res_iepf_, const string& type_){
	TipLines3_xy res_iepf_first = res_iepf_.first;
	size_t_vv res_iepf_second = res_iepf_.second;
	
	Landmark ObsLmkFitLinear;
	LVVLinear LSLinear;
	ObsLmkFitLinear.resize(res_iepf_second.size());
	
	LSLinear.resize(res_iepf_.first.size());
	float alpha_linear;
	for(size_t segment=0; segment<res_iepf_.first.size(); segment++){
		TipLiness_xy seg_part_ = res_iepf_.first[segment];
		size_t_v ind_seg_part = res_iepf_.second[segment];
		LSLinear[segment].resize(seg_part_.size());
		ObsLmkFitLinear[segment].resize(seg_part_.size()-1);
		bool awal_ = true;
		bool kedua_ = true;
		
		float theta_1, theta_biasa;
		float m_1;

		vector<Mat> mc_lines;
		vector<Vec2f> xsxf; //xsxf.resize(seg_part_.size());

		// cout << "ngaris: " << seg_part_.size() << endl;
		Mat tampil_mc(img_laser.size(), CV_8UC3, Scalar::all(255));

		for(size_t garis=0; garis<seg_part_.size(); garis++){
			TipLines_xy garis_part_ = seg_part_[garis];
			LSLinear[segment][garis].yi = Mat::zeros(garis_part_.size(), 1, CV_32FC1);
			LSLinear[segment][garis].A = Mat::ones(garis_part_.size(), 2, CV_32FC1);

			for(size_t i=0; i<garis_part_.size(); i++){
				LSLinear[segment][garis].A.at<float>(i,0) = garis_part_[i].first;
				LSLinear[segment][garis].yi.at<float>(i) = garis_part_[i].second;
			}

			float seperN = float(1.0/garis_part_.size());

			Mat A_ = LSLinear[segment][garis].A;
			LSLinear[segment][garis].mc = (A_.t() * A_).inv() * A_.t() * LSLinear[segment][garis].yi;

			mc_lines.emplace_back(LSLinear[segment][garis].mc);

			xsxf.emplace_back(Vec2f(garis_part_[0].first, garis_part_[garis_part_.size()-1].first));
			Point2f tip_point = Point2f(res_iepf_first_const[ind_seg_part[garis]].first, res_iepf_first_const[ind_seg_part[garis]].second);
			if(awal_){
				m_1 = LSLinear[segment][garis].mc.at<float>(0);
				awal_ = false;
			}else{
				circle(img_laser, tip_point, 10, Scalar(0,0,255), 3);
				ObsLmkFitLinear[segment][garis-1][0] = res_iepf_first_const[ind_seg_part[garis]].first;
				ObsLmkFitLinear[segment][garis-1][1] = res_iepf_first_const[ind_seg_part[garis]].second;
				float numer_ = m_1 - LSLinear[segment][garis].mc.at<float>(0);
				float denum_ = (1.0 + m_1*LSLinear[segment][garis].mc.at<float>(0));
				alpha_linear = fabs(atan2(numer_, denum_));
				
				string sign_numer, sign_denum;
				if(numer_< 0.0) 
					sign_numer = "-";
				else sign_numer = "+";

				if(denum_ < 0.0)
					sign_denum = "-";
				else sign_denum = "+";

				if(((sign_numer == "+") && (sign_denum == "+")) || ((sign_numer == "+") && sign_denum == "-"))
					alpha_linear = M_PI - alpha_linear;
				ObsLmkFitLinear[segment][garis-1][2] = alpha_linear;

				// cout << "sign numer denum: " << sign_numer << "\t" << sign_denum << endl;
				// cout << "alpha_linear: " << alpha_linear * 180.0/M_PI << endl;
				m_1 = LSLinear[segment][garis].mc.at<float>(0);
			}
		}

		for(size_t sx = 0; sx <xsxf.size(); sx++){
			float m_ = mc_lines[sx].at<float>(0);
			float c_ = mc_lines[sx].at<float>(1);
			float x0 = xsxf[sx][0];

			if(x0 < xsxf[sx][1])
				for(; x0<= xsxf[sx][1]; x0++){
					float y = m_*x0 + c_;
					circle(tampil_mc, Point2f(x0, y), 3, Scalar(255,0,0), -1);
				}
			else
				for(; x0> xsxf[sx][1]; x0--){
					float y = m_*x0 + c_;
					circle(tampil_mc, Point2f(x0, y), 3, Scalar(255,0,0), -1);
				}
		}
		namedWindow("tampil_mc", CV_WINDOW_NORMAL);
		imshow("tampil_mc", tampil_mc);
	}
	return ObsLmkFitLinear;
}
/********************************* END OF PROSES DATA LASER ************************************/

void Pioneer::Observation(){
  DT = time_before - ros::Time::now().toSec();
  position_dr.x = position_dr_awal.x + velocity_xy.x*DT;
  position_dr.y = position_dr_awal.y + velocity_xy.y*DT;
  orientation_dr = orientation_dr_awal + velocity_w*DT;

  position_dr_awal.x = position_dr.x;
  position_dr_awal.y = position_dr.y;
  orientation_dr_awal = orientation_dr;
  time_before = ros::Time::now().toSec();
}


void Pioneer::getVelocity(const geometry_msgs::Vector3::ConstPtr& vel_lin_ang_){
	velocity_xy.x = float(FRAME_WIDTH / FIELD_WIDTH) * vel_lin_ang_->x;
	velocity_xy.y = -float(FRAME_WIDTH / FIELD_WIDTH) * vel_lin_ang_->y;
	velocity_w = vel_lin_ang_->z;

	input_loc.at<float>(0) = sqrt(pow(velocity_xy.x, 2) + pow(velocity_xy.y, 2));
	input_loc.at<float>(1) = velocity_w;
}

void Pioneer::getPos(const geometry_msgs::Vector3::ConstPtr& pos_orient_){
	position.x = pos_orient_->x;
	position.y = pos_orient_->y;
	orientation = 180.0 -  pos_orient_->z;
	
    posNow = position;

	cvt2frameWorld(position);
    orientation = deg2rad(orientation);

    orientation_plan = deg2rad(pos_orient_->z);

    if(isAwal){
    	position_est_awal.x = position_dr_awal.x = position.x;
    	position_est_awal.y = position_dr_awal.y = position.y;
    	orientation_est_awal = orientation_dr_awal = orientation;
    	isAwal = false;
    }
}

/************************************* ODOMETRY MOTION MODEL ***********************************/
void Pioneer::ekf_localization_odom(const Landmark& obs_landmark, const Landmark_single& true_landmark_){
  if(estAwal){
  	position_est.x = position_dr.x;
  	position_est.y = position_dr.y;
  	orientation_est = orientation_dr;
  	estAwal = false;
  	return;
  }

  drot1 = atan2((position_est.y - position_est_awal.y),(position_est.x - position_est_awal.x)) - orientation_est;
  dtrans = sqrt(pow(position_est_awal.x - position_est.x, 2) + pow(position_est_awal.y - position_est.y, 2));
  drot2 = orientation_est - orientation_est_awal - drot1;

  float del_x = dtrans*cos(orientation_est_awal + drot1);
  float del_y = dtrans*sin(orientation_est_awal + drot1);

  position_est.x = position_est_awal.x + del_x;
  position_est.y = position_est_awal.y + del_y;
  orientation_est = drot1 + drot2;

  Mat Gt  = (Mat_<float>(3,3) << 1.0, 0.0, -del_y,
  								 0.0, 1.0, del_x,
  								 0.0, 0.0, 1.0);

  Mat Vt = (Mat_<float>(3,3) << -del_y, del_x/dtrans, 0.0,
  								del_x, del_y/dtrans, 0.0,
  								1.0, 0.0, 1.0);

  Mat Mt  = (Mat_<float>(3,3) << alpha_1*fabs(drot1) + alpha_2*dtrans, 0.0, 0.0,
  								 0.0, alpha_3*dtrans + alpha_4*(fabs(drot1) + fabs(drot2)), 0.0,
  								 0.0, 0.0, alpha_1*fabs(drot1) + alpha_2*dtrans);

  Omegat = Gt*Omegat*Gt.t() + Vt*Mt*Vt.t();

  Mat K_iz = Mat::zeros(3,1,CV_32FC1);
  Mat K_ih = Mat::zeros(3,3,CV_32FC1);

  for(size_t tr=0; tr<true_landmark_.size(); tr++){
  	Mat delta(2,1,CV_32FC1);
	delta.at<float>(0) = true_landmark_[tr][0] - position_est.x;
	delta.at<float>(1) = true_landmark_[tr][1] - position_est.y;
	Mat qM = delta.t() * delta;
	float q = qM.at<float>(0);

	Mat z_hat_i  = (Mat_<float>(3,1) << sqrt(q),
    		   							atan2(true_landmark_[tr][0] - position_est.y, true_landmark_[tr][0] - position_est.x) - orientation_est,
    		   							true_landmark_[tr][2]);

	for(size_t j=0; j<obs_landmark.size(); j++){
	   for(size_t i=0; i<obs_landmark[j].size(); i++){
	   		// data observasi
	   		Mat delta_(2,1,CV_32FC1);
	   		delta_.at<float>(0) = obs_landmark[j][i][0] - position_est.x;
	   		delta_.at<float>(1) = obs_landmark[j][i][1] - position_est.y;
	   		Mat q_M = delta_.t() * delta_;
	   		float q_ = q_M.at<float>(0);

	   		Mat z_i = (Mat_<float>(3,1) << sqrt(q_),
	   			   						atan2(obs_landmark[j][i][1] - position_est.y, obs_landmark[j][i][0] - position_est.x) - orientation_est,
	   			   						obs_landmark[j][i][2]);
	   		Mat H_i = (Mat_<float>(3,3) << z_i.at<float>(0)*delta_.at<float>(0), -z_i.at<float>(0)*delta_.at<float>(1), 0.0,
	   			   						   delta_.at<float>(1), delta_.at<float>(0), -1.0,
	   			   						   0.0, 0.0, 0.0);
	   		Mat S = H_i*Omegat*H_i.t() + Qt;
	   		Mat K_i_temp = Omegat*H_i.t()*S.inv();
	   		K_iz += K_i_temp*(z_i - z_hat_i);
	   		K_ih += K_i_temp*H_i;
	   }
	}
  }

  position_est_awal.x = position_est.x + K_iz.at<float>(0);
  position_est_awal.y = position_est.y + K_iz.at<float>(1);
  Omegat = (I - K_ih)*Omegat;
}
/************************************** END OF ODOMETRY MOTION MODEL **********************************************************/

/******************************************** VELOCITY MOTION MODEL ************************************/
void Pioneer::ekf_localization_velo(const Landmark& obs_landmark, const Landmark_single& true_landmark_){
	if(estAwal){
		estAwal = false;
		return;
	}

	float dx, dy, dw;

	// cout << "v: " << input_loc.at<float>(0) << "\t w: " << input_loc.at<float>(1) << endl << endl;
	if(input_loc.at<float>(1) > 1e-2){
		dx = input_loc.at<float>(0)/input_loc.at<float>(1) * (-sin(orientation_est_awal)
									 + sin(orientation_est_awal + input_loc.at<float>(1)*DT));
		dy = input_loc.at<float>(0)/input_loc.at<float>(1) * (cos(orientation_est_awal)
									 - cos(orientation_est_awal + input_loc.at<float>(1)*DT));
	}else{
		dx =  input_loc.at<float>(0) * DT *cos(orientation_est_awal);
		dy = input_loc.at<float>(0) * DT * sin(orientation_est_awal);
	}

	dw = input_loc.at<float>(1) * DT;

	position_est.x = position_est_awal.x + dx;
	position_est.y = position_est_awal.y + dy;
	orientation_est = orientation_est_awal + dw;

	Mat Gt  = (Mat_<float>(3,3) << 1.0, 0.0, dy,
  								 0.0, 1.0, -dx,
  								 0.0, 0.0, 1.0);

	Omegat = Gt*Omegat*Gt.t();

	Mat K_iz = Mat::zeros(3,1,CV_32FC1);
  	Mat K_ih = Mat::zeros(3,3,CV_32FC1);

  for(size_t tr=0; tr<true_landmark_.size(); tr++){
  	Mat delta(2,1,CV_32FC1);
	delta.at<float>(0) = true_landmark_[tr][0] - position_est.x;
	delta.at<float>(1) = true_landmark_[tr][1] - position_est.y;
	Mat qM = delta.t() * delta;
	float q = qM.at<float>(0);

	Mat z_hat_i  = (Mat_<float>(3,1) << sqrt(q),
    		   							atan2(true_landmark_[tr][0] - position_est.y, true_landmark_[tr][0] - position_est.x)
    		   								 - orientation_est,
    		   							true_landmark_[tr][2]);

	for(size_t j=0; j<obs_landmark.size(); j++){
	   for(size_t i=0; i<obs_landmark[j].size(); i++){
	   		// data observasi
	   		Mat delta_(2,1,CV_32FC1);
	   		delta_.at<float>(0) = obs_landmark[j][i][0] - position_est.x;
	   		delta_.at<float>(1) = obs_landmark[j][i][1] - position_est.y;
	   		Mat q_M = delta_.t() * delta_;
	   		float q_ = q_M.at<float>(0);

	   		Mat z_i = (Mat_<float>(3,1) << sqrt(q_),
	   			   						atan2(obs_landmark[j][i][1] - position_est.y, obs_landmark[j][i][0] - position_est.x) - orientation_est,
	   			   						obs_landmark[j][i][2]);
	   		Mat H_i = (Mat_<float>(3,3) << delta_.at<float>(0)/sqrt(q_), -delta_.at<float>(1)/sqrt(q_), 0.0,
	   			   						   delta_.at<float>(1)/q_, delta_.at<float>(0)/q_, -1.0/q_,
	   			   						   0.0, 0.0, 0.0);
	   		Mat S = H_i*Omegat*H_i.t() + Qt;
	   		Mat K_i_temp = Omegat*H_i.t()*S.inv();
	   		K_iz += K_i_temp*(z_i - z_hat_i);
	   		K_ih += K_i_temp*H_i;
	   }
	}
  }

  position_est_awal.x = position_est.x + K_iz.at<float>(0);
  position_est_awal.y = position_est.y + K_iz.at<float>(1);
  orientation_est_awal = orientation_est + K_iz.at<float>(2);
  Omegat = (I - K_ih)*Omegat;
}
/***************************************END OF VELOCITY MOTION MODEL **********************************/


void Pioneer::show(){
  circle(img_bck_clone, position, 1, Scalar(0,255,0), 1);
  circle(img_bck_clone, position_dr, 1, Scalar(0,0,255), 1);
  circle(img_bck_clone, Point2f(fabs(position_est.x), fabs(position_est.y)), 1, Scalar(255,0,255), 1);

  img_bck = img_bck_clone.clone();

  tujuan_xy.x = position.x +  15.0 * cos(orientation);
  tujuan_xy.y = position.y +  15.0 * sin(orientation);  
  circle(img_bck, position, 5, Scalar(0,255,0), -1);
  line(img_bck, position, tujuan_xy, Scalar(255,255,0), 2);

  tujuan_xy_dr.x = position_dr.x +  15.0 * cos(orientation_dr);
  tujuan_xy_dr.y = position_dr.y +  15.0 * sin(orientation_dr);
  circle(img_bck, position_dr, 5, Scalar(0,0,255), -1);
  line(img_bck, position_dr, tujuan_xy_dr, Scalar(255,0,0), 2);

  tujuan_xy_est.x = fabs(position_est.x) +  15.0 * cos(orientation_est);
  tujuan_xy_est.y = fabs(position_est.y) +  15.0 * sin(orientation_est);
  circle(img_bck, Point2f(fabs(position_est.x), fabs(position_est.y)), 5, Scalar(255,0,255), -1);
  line(img_bck, Point2f(fabs(position_est.x), fabs(position_est.y)), tujuan_xy_est, Scalar(0,0,0), 2);

  // pengamatan data laser terhadap dead reckoning robot
  circle(img_laser, position_dr, 5, Scalar(0,0,255), -1);
  line(img_laser, position_dr, tujuan_xy_dr, Scalar(255,0,0), 2);

  cout << "position_true: " << position     << "\t orientation_true: " << orientation *180.0/M_PI  << endl
  	   << "position_dr  : " << position_dr  << "\t orientation_dr: " << orientation_dr *180.0/M_PI << endl;
  cout << "position_est : " << Point2f(fabs(position_est.x), fabs(position_est.y)) 
  	   << "\t orientation_est: " << orientation_est *180.0/M_PI
  	   << endl << endl;
  
  cv::namedWindow("img_laser", CV_WINDOW_NORMAL);
  cv::imshow("img_laser", img_laser);
  cv::namedWindow("img_bck", CV_WINDOW_NORMAL);
  cv::imshow("img_bck", img_bck);
  cv::waitKey(1);
}

void Pioneer::loop(){
	state2hover_.data = false;

	state2hover_pub.publish(state2hover_);

	img_laser = Mat(img_bck.size(), CV_8UC3, Scalar::all(255));
	start_path = position;
	finish_path = quad_position;
	// finish_path = Point2f(-1.5000e+00, -1.5750e+00);
	cvt2frameWorld(finish_path);
	
	selfSetActuator();
	process_laser_data();
	Observation();
	ekf_localization_velo(observed_landmark, true_landmark);
	show();
	sleep_(5);
}