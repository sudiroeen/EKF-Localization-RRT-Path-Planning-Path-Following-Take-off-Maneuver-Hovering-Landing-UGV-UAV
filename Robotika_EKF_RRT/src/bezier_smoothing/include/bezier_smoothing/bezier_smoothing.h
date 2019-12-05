/*
	Copyright by:
		Sudiro
			[at] SudiroEEN@gmail.com
	
	This is my library for RRT Path Planniing Algorithm

	Available on my site:
		github.com/sudiroeen
*/

#ifndef PATH_SMOOTHING
#define PATH_SMOOTHING

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef vector<Point2f> PATH;

class PathSmoothing{
   public:
    PATH BezierSpline(PATH fixed_path_, Mat& img_smooth_);
};

#endif
