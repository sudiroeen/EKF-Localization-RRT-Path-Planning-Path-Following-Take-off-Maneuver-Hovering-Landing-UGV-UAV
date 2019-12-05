/*
	Copyright by:
		Sudiro
			[at] SudiroEEN@gmail.com
	
	This is my library for RRT Path Planniing Algorithm

	Available on my site:
		github.com/sudiroeen
*/

#ifndef RRT_H
#define RRT_H

#include <bezier_smoothing/bezier_smoothing.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

#define TUGAS_ROBOTIKA

typedef vector<vector<Point2f> > KONTUR;
typedef vector<vector<Point2f> > TREE;
typedef vector<Point2f> PATH;
typedef vector<vector<Point2f> > OBSTACLE;

class PathPlanning{
public:
	PathPlanning();
	PathPlanning(Point2f start_, Point2f finish_, OBSTACLE _obs_);
	void execute();
private:
	bool check_intersect(const KONTUR& obs_, vector<Point2f> sf_line);
	void getFixedPathFunc(PATH& toIsi, const PATH& pathFull, const vector<int>& ind_parent_, size_t id_p);
	PATH RRT(const KONTUR& obstacle_, const Point2f& start_, const Point2f& finish_);
#ifdef MANUAL	
	void mouseCb(int event, int x, int y, int flags, void* userdata);
#endif

private:
	Point2f start;
  	Point2f finish;
	string windowName;
	Mat img;
	Mat img_hasil;
	Mat img_smooth;
	Mat img_hitam;
	Mat frame_obs;
	bool tekan;
	OBSTACLE obstacle;
	TREE res_tree;

public:
	PATH getPathRRT;
};

#endif
