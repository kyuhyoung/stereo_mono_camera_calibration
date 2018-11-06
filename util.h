#ifndef _UTIL_H_
#define _UTIL_H_

#include <string>

std::string python_join_equivalent(const std::string& dir_to_file, const std::string& filename);

void error(const char *);

double what_time_in_seconds_is_it_now();

bool mkdir_if_not_exist(const char *dir);

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;
using namespace std;
bool retrieve_left_and_right_images_ocv(Mat& img_l, Mat& img_r, VideoCapture& cap, bool is_gray);

void merge_mat_side_by_side(Mat& m_full, Mat& m_l, Mat& m_r);

void get_cam_properties_ocv(int& w, int& h, int& fps, const VideoCapture *cap);

void set_cam_properties_ocv(VideoCapture *cap, const int& w, const int& h, const int& fps);

bool check_if_request_is_accepted_ocv(const VideoCapture *cap, const int& w, const int& h, const int& fps);

bool save_images(const vector<string>& li_fn_img, const int idx_start, const Mat& mat_l, const Mat& mat_r, const string& dir_img);




#endif
