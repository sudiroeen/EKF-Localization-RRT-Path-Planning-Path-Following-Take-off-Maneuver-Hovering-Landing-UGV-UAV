/*
	Copyright by:
		Sudiro
			[at] SudiroEEN@gmail.com
	
	This is my library for RRT Path Planniing Algorithm

	Available on my site:
		github.com/sudiroeen [as soon as uploaded]
*/

#include <rrt/rrt.h>

PathPlanning::PathPlanning(Point2f start_, Point2f finish_, OBSTACLE _obs_)
			: tekan(false), start(start_), finish(finish_), obstacle(_obs_),
			  windowName("RRT - [Rapidly-exploring Random Tree]"),
			  img(Mat(600, 600, CV_8UC3, Scalar::all(255))),
			  img_hasil(Mat(600, 600, CV_8UC3, Scalar::all(255))),
			  img_smooth(Mat(600, 600, CV_8UC3, Scalar::all(255)))
{
	line(img, _obs_[0][0], _obs_[0][1], Scalar::all(0), 10);
	line(img, _obs_[0][1], _obs_[0][2], Scalar::all(0), 10);
	line(img, _obs_[0][2], _obs_[0][3], Scalar::all(0), 10);
	line(img, _obs_[0][3], _obs_[0][0], Scalar::all(0), 10);

	line(img, _obs_[1][0], _obs_[1][1], Scalar::all(0), 10);
	line(img, _obs_[1][1], _obs_[1][2], Scalar::all(0), 10);	

	line(img_hasil, _obs_[0][0], _obs_[0][1], Scalar::all(0), 10);
	line(img_hasil, _obs_[0][1], _obs_[0][2], Scalar::all(0), 10);
	line(img_hasil, _obs_[0][2], _obs_[0][3], Scalar::all(0), 10);
	line(img_hasil, _obs_[0][3], _obs_[0][0], Scalar::all(0), 10);

	line(img_hasil, _obs_[1][0], _obs_[1][1], Scalar::all(0), 10);
	line(img_hasil, _obs_[1][1], _obs_[1][2], Scalar::all(0), 10);
}

#ifdef MANUAL
void PathPlanning::mouseCb(int event, int x, int y, int flags, void* userdata){
	if(event == EVENT_LBUTTONDOWN){
		tekan = true;
	}

	if(tekan){
		obstacle.emplace_back(Point2f(x,y));
		circle(img, Point2f(x,y), 8, Scalar::all(0), -1);
	}

	if(event == EVENT_LBUTTONUP){
		tekan = false;
	}
}
#endif

bool PathPlanning::check_intersect(const KONTUR& obs_, vector<Point2f> sf_line){
	Mat zero1 = Mat::zeros(img.size(), CV_8UC1);
	Mat zero2 = Mat::zeros(img.size(), CV_8UC1);

#ifdef MANUAL
	for(size_t k=0; k<obs_[0].size(); k++){
		circle(zero1, obs_[0][k], 8, uchar(255), -1);
	}
#elif defined(TUGAS_ROBOTIKA)
	line(zero1, obs_[0][0], obs_[0][1], uchar(255), 15);
	line(zero1, obs_[0][1], obs_[0][2], uchar(255), 15);
	line(zero1, obs_[0][2], obs_[0][3], uchar(255), 15);
	line(zero1, obs_[0][3], obs_[0][0], uchar(255), 15);

	for(size_t t=0; t<obs_[1].size()-1; t++)
		line(zero1, obs_[1][t], obs_[1][t+1], uchar(255), 15);
#endif
	line(zero2, sf_line[0], sf_line[1], uchar(255), 15);

	Mat res_bit_and; 
	bitwise_and(zero1, zero2, res_bit_and);
	int res_and = (int)sum(res_bit_and)[0];
	if(res_and > 0)
		return true;
	else
		return false;
}

void PathPlanning::getFixedPathFunc(PATH& toIsi, const PATH& pathFull, const vector<int>& ind_parent_, size_t id_p){
	if(ind_parent_[id_p] == -1){
		toIsi.insert(toIsi.begin(), pathFull[0]);
		return;
	}
	toIsi.insert(toIsi.begin(), pathFull[id_p]);
	getFixedPathFunc(toIsi, pathFull, ind_parent_, ind_parent_[id_p]);
}

PATH PathPlanning::RRT(const KONTUR& obstacle_, const Point2f& start_, const Point2f& finish_){
	PATH topath;
	topath.emplace_back(start_);
	topath.emplace_back(finish_);

	vector<int> ind_parent;
	ind_parent.emplace_back(-1);

	if(!check_intersect(obstacle_, topath)){
		cout << "langsung return" << endl;
		line(img, start_, finish_, Scalar(45, 150, 44), 2);
		return topath;
	}

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> rand_valx(130,430	);
	std::uniform_int_distribution<> rand_valy(200,385);
	while(true){
		int rand_x = rand_valx(gen);
		int rand_y = rand_valy(gen);
		circle(img, Point2f(rand_x, rand_y), 1, Scalar(0,0,255), -1);
		float dist0 = sqrt(pow(float(start_.x - rand_x), 2) + pow(float(start_.y - rand_y), 2));
		size_t before_end = topath.size()-1;
		int parent_id = 0;
		for(size_t i=0; i<before_end; i++){
			vector<Point2f> temp_sfline;
			temp_sfline.emplace_back(topath[i]);
			temp_sfline.emplace_back(Point2f(rand_x, rand_y));
			if(!check_intersect(obstacle_, temp_sfline)){
				float dist_to_rand = sqrt(pow(float(topath[i].x - rand_x), 2.0) + pow(float(topath[i].y - rand_y),2.0));
				if(dist_to_rand <= dist0){
					parent_id = i;
					dist0 = dist_to_rand;
				}
			}else{
				parent_id = -1;
			}
		}
		if(parent_id != -1){
			line(img, topath[parent_id], Point2f(rand_x, rand_y), Scalar(45,150,44), 2);
			topath.insert(topath.end()-1, Point2f(rand_x, rand_y));
			ind_parent.emplace_back(parent_id);
			vector<Point2f> temp_sfline;
			temp_sfline.emplace_back(finish_);
			temp_sfline.emplace_back(Point2f(rand_x, rand_y));
			if(!check_intersect(obstacle_, temp_sfline)){
				ind_parent.emplace_back(before_end);
				line(img, topath[before_end], finish_, Scalar(45,150,44), 2);
				break;
			}
		}
		imshow(windowName, img);
		waitKey(100);
	}

	PATH fixed_path;
	fixed_path.emplace_back(finish_);
	getFixedPathFunc(fixed_path, topath, ind_parent, ind_parent[ind_parent.size() - 1]);
	for(size_t k=1; k<fixed_path.size(); k++){
		line(img_hasil, fixed_path[k-1], fixed_path[k], Scalar(255,255,0), 2);
	}
	namedWindow("path_hasil", CV_WINDOW_NORMAL);
	imshow("path_hasil", img_hasil);
	return fixed_path;
}

void PathPlanning::execute(){
  res_tree.resize(1);
  res_tree[0].emplace_back(start);
  res_tree[0].emplace_back(finish);
  namedWindow(windowName, CV_WINDOW_NORMAL);
  circle(img, start, 5, Scalar(255,0,0), -1);
  circle(img, finish, 5, Scalar(191,0,255), -1);
  circle(img_hasil, start, 5, Scalar(255,0,0), -1);
  circle(img_hasil, finish, 5, Scalar(191,0,255), -1);

  cout << "start: " << start << "\t finish: " << finish << endl;
#ifdef MANUAL
  setMouseCallback(windowName, mouseCb, NULL);
  while(true){
  	imshow(windowName, img);
  	if(waitKey(1) == 27)
  		break;
  }
#endif
  // cout << "obstacle.size: " << obstacle[0].size() << "\t" << obstacle[1].size() << endl;
  if(obstacle.size() != 0){
	  OBSTACLE obstacle_v;

#ifdef MANUAL
	  obstacle_v.emplace_back(obstacle);
#elif defined(TUGAS_ROBOTIKA)
	  obstacle_v = obstacle;
#endif
	  getPathRRT = RRT(obstacle_v, start, finish);

	  cout << "getPathRRT: " << endl;
	  for(size_t s=0; s<getPathRRT.size(); s++)
	  	cout << getPathRRT[s] << endl;
	  
	  PathSmoothing Smooth;	  
	  PATH smooth_path = Smooth.BezierSpline(getPathRRT, img_smooth);

	  namedWindow("[failed] Bezier Spline", CV_WINDOW_NORMAL);
	  imshow("[failed] Bezier Spline", img_smooth);
	  imshow(windowName, img);
	}
	waitKey(0);
}
