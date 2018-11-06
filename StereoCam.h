#ifndef _STEREO_CAM_H_
#define _STEREO_CAM_H_

#include "withrobot_camera.hpp"
#include <opencv2/opencv.hpp>

class StereoCam
{
public:
    StereoCam();
    StereoCam(bool is_ocam, /*std::string& id_cam,*/ int idx_cam, cv::Size& saiz, int fps);
    ~StereoCam();
    bool is_opened();
    bool set_properties(cv::Size& saiz, int fps);
    bool read_left_right(cv::Mat& mat_full, cv::Mat& mat_l, cv::Mat& mat_r);
    bool is_ocams();
    void create(bool is_ocam, /*std::string& id_cam,*/ int idx_cam, cv::Size& saiz, int fps);
    cv::Size get_camera_resolution();
private:
	void compute_camera_resolution_and_fps();
    bool check_if_request_is_accepted(const cv::Size& saiz, const int fps);
    //int get_properties(std::string);
    //void set_camera_id
    cv::VideoCapture *m_cap_ocv;
    cv::Mat m_mat_src_ocams, m_mat_dst_ocams[2];//, m_mat_ocv;//, m_mat;
    Withrobot::Camera *m_cap_ocams;
    bool m_is_ocams;
	bool m_is_mono;
    //std::string m_id_cam;
    cv::Size m_size;
	int m_frame_rate;
    int m_image_size;
};

#endif         
