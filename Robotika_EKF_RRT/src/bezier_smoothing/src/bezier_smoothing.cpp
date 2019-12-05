/*

Copyright by 
	Sudiro 
		[at] SudiroEEN@gmail.com
	guthub.com/sudiroeen

*/

#include "bezier_smoothing/bezier_smoothing.h"

PATH PathSmoothing::BezierSpline(PATH fixed_path_, Mat& img_smooth_){
	// -1.0000e+00, 0, 0
	// -1.4901e-08, +5.2500e-01, 0
	// +1.0000e+00, -5.0000e-01, 0
	// +1.5000e+00, +4.5000e-01, 0
	PATH smoothedPath;
	fixed_path_.insert(fixed_path_.begin(), Point2f(fixed_path_[0].x+1, fixed_path_[0].y+1));
	fixed_path_.insert(fixed_path_.end(), Point2f(fixed_path_[fixed_path_.size()-2].x+1, fixed_path_[fixed_path_.size()-2].y+1));
	Mat M_Bezier = (Mat_<float>(4,4) << -1, 3, -3, 1,
										 3, -6, 3, 0,
										 -3, 3, 0, 0,
										 1, 0, 0, 0);
	float max_step_u = 15.0;
	bool isAwal = true;
	for(size_t s=1; s<fixed_path_.size()-2; s++){
		Mat Posisi = (Mat_<float>(4,2) << fixed_path_[s-1].x, fixed_path_[s-1].y,
										  fixed_path_[s].x, fixed_path_[s].y,
										  fixed_path_[s+1].x, fixed_path_[s+1].y,
										  fixed_path_[s+2].x, fixed_path_[s+2].y);
		if(isAwal){
			for(float step_u=0.0; step_u<=max_step_u; step_u+=1.0){
				float u = step_u/max_step_u;
				Mat seq_u = (Mat_<float>(1,4) << pow(u, 3.0), pow(u, 2.0), u, 1.0);
				Mat hasil_ = seq_u * M_Bezier * Posisi;
				smoothedPath.emplace_back(Point2f(hasil_.at<float>(0), hasil_.at<float>(1)));
			}
			isAwal = false;
		}else{
			for(float step_u=10.0; step_u<=max_step_u; step_u+=1.0){
				float u = step_u/max_step_u;
				Mat seq_u = (Mat_<float>(1,4) << pow(u, 3.0), pow(u, 2.0), u, 1.0);
				Mat hasil_ = seq_u * M_Bezier * Posisi;
				smoothedPath.emplace_back(Point2f(hasil_.at<float>(0), hasil_.at<float>(1)));
			}
		}
	}

	for(size_t r=1; r<smoothedPath.size(); r++){
		cout << "smoothedPath: " << smoothedPath[r] << endl;
		line(img_smooth_, smoothedPath[r-1], smoothedPath[r], Scalar::all(0), 2);
	}
	return smoothedPath;
}
