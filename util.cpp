#include "util.h"

#include <iostream>
#include <experimental/filesystem>
std::string python_join_equivalent(const std::string& dir_to_file, const std::string& filename)
{
    //std::cout << dir_to_file << " " << filename << std::endl;
    std::experimental::filesystem::path dir(dir_to_file);
    std::experimental::filesystem::path fn(filename);
    std::experimental::filesystem::path full_path = dir / fn;
    return full_path.u8string();
}



#include <stdlib.h>
#include <cstdio>
#include <cassert>
void error(const char *s)
{
    perror(s);
    assert(0);
    exit(-1);
}


//------------ Get the current time in the unit of second --------------
//  original code : https://github.com/pjreddie/darknet/blob/master/src/utils.c
//#include <time.h>
#include <sys/time.h>
#include <unistd.h>
double what_time_in_seconds_is_it_now()
{
    struct timeval time;
    if (gettimeofday(&time, NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}


#include <sys/stat.h>
#include <stdio.h>


bool mkdir_if_not_exist(const char *dir)
{
    bool is_folder_exist = false;
    struct stat st;
    if(0 == stat(dir, &st))
    {
        if(0 != (st.st_mode & S_IFDIR))
        {
            is_folder_exist = true;
            printf("%s DOES exist. \n", dir);
        } 
    }
    if(!is_folder_exist)
    {
        int nError = 0;
#if defined(_WIN32)
        nError = _mkdir(dir);
#else
        mode_t nMode = 0733;    //UNIX style permission
        nError = mkdir(dir, nMode);
#endif
        if(0 != nError)
        {
            // handle your error
            printf("Can NOT make a directory %s\n", dir);
        }
        else
        {
            is_folder_exist = true;
            printf("Just created a directory %s\n", dir);
        }
    }
    return is_folder_exist;
}


//#include "opencv2/highgui.hpp"
bool retrieve_left_and_right_images_ocv(Mat& img_l, Mat& img_r, VideoCapture& cap, bool is_gray)
{
	cv::Mat frame;
	cap >> frame;
	if (is_gray) cvtColor(frame, frame, CV_BGR2GRAY);
	img_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
	img_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
	return true;
}

void merge_mat_side_by_side(Mat& m_full, Mat& m_l, Mat& m_r)
{
	if(m_full.empty())
	{
		m_full.create(m_l.rows, m_l.cols + m_r.cols, m_l.type());
	}
	m_full.adjustROI(0, 0, 0, -m_r.cols);
	m_l.copyTo(m_full);
	m_full.adjustROI(0, 0, -m_l.cols, m_r.cols);
	m_r.copyTo(m_full);
	m_full.adjustROI(0, 0, m_l.cols, 0);
	return;
}

void get_cam_properties_ocv(int& w, int& h, int& fps, const VideoCapture *cap)
{
	w = (int)cap->get(CV_CAP_PROP_FRAME_WIDTH);
	h = (int)cap->get(CV_CAP_PROP_FRAME_HEIGHT);
	fps = (int)cap->get(CV_CAP_PROP_FPS);
	return; 
}

bool check_if_request_is_accepted_ocv(const VideoCapture *cap, const int& w_requested, const int& h_requested, const int& fps_requested)
{
	int w_actual, h_actual, fps_actual;
	get_cam_properties_ocv(w_actual, h_actual, fps_actual, cap);
	printf(
		"width : %d (requested), %d (actual) \nheight : %d (requested), %d (actual) \nfps : %d (requested), %d (actual)\n", w_requested, w_actual, h_requested, h_actual, fps_requested, fps_actual); 
	bool is_accepted = w_requested == w_actual && h_requested == h_actual && fps_requested == fps_actual;
	return is_accepted;
}

void set_cam_properties_ocv(VideoCapture *cap, const int& wid, const int& hei, const int& fps)
{
	cap->grab();
	cap->set(CV_CAP_PROP_FRAME_WIDTH, wid);
	cap->set(CV_CAP_PROP_FRAME_HEIGHT, hei);
	cap->set(CV_CAP_PROP_FPS, fps);	
	return;
}



bool save_images(const vector<string>& li_fn_img, const int idx_start, const Mat& mat_l, const Mat& mat_r, const string& dir_img)
{
	bool is_mono = mat_r.empty();
	int n_save = is_mono ? 1 : 2;
	for(int k = 0; k < n_save; k++)
	{
		bool is_saved = false;
		//const string& filename = param_save->imagelist[n_img_saved * 2 + k];
		const string& filename = li_fn_img[idx_start + k];
		//string fn_raw = python_join_equivalent(param_save->dir_img, filename);
		string fn_raw = python_join_equivalent(dir_img, filename);
		try
		{
			is_saved = imwrite(fn_raw, k ? mat_r : mat_l);
		}
		catch (runtime_error& ex)
		{
			fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
			exit(0);
		}
		if (is_saved)
		{
			cout << "Just saved " << fn_raw << endl;
		}
		else
		{
			cout << "Can NOT save as " << fn_raw << endl;
			exit(0);
		}
	}
	return true;
}




