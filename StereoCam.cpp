#include "StereoCam.h"
#include "util.h"

/*
void merge_mat_side_by_side(cv::Mat& m_full, cv::Mat& m_l, cv::Mat& m_r)
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
*/

StereoCam::StereoCam()
{
    m_cap_ocams = 0;    m_cap_ocv = 0;
    m_image_size = -1;
}

StereoCam::StereoCam(bool is_ocam, /*std::string& id_cam,*/ int idx_cam, cv::Size& saiz, int fps)
{
	create(is_ocam, idx_cam, saiz, fps);
}

void StereoCam::create(bool is_ocam, /*std::string& id_cam,*/ int idx_cam, cv::Size& saiz, int fps)
{
    //m_id_cam = id_cam;
    m_is_ocams = is_ocam;
    m_cap_ocams = 0;    m_cap_ocv = 0;
    m_image_size = -1;
    if(is_ocams())
    {
        std::string devPath = "/dev/video" + std::to_string(idx_cam);
        //std::cout << "StereoCam 000" << std::endl;
        m_cap_ocams = new Withrobot::Camera(devPath.c_str());
        //std::cout << "StereoCam 111" << std::endl;
        if(!m_cap_ocams)
        {
            //std::cout << "Can NOT open the camera : " << id_cam << " at " << devPath << std::endl;
            std::cout << "Can NOT open the camera : " << " at " << devPath << std::endl;
            exit(0);
        }
        set_properties(saiz, fps);
        Withrobot::camera_format camFormat;
        m_cap_ocams->get_current_format(camFormat);
        std::string camName = m_cap_ocams->get_dev_name();
        std::string camSerialNumber = m_cap_ocams->get_serial_number();
        printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
        printf("----------------- Current format informations -----------------\n");
        camFormat.print();
        printf("Before Gain : %d, Exposure (Absolute) : %d\n", m_cap_ocams->get_control("Gain"), m_cap_ocams->get_control("Exposure (Absolute)"));
        m_cap_ocams->set_control("Exposure (Absolute)", 200);
        printf("Aftere Gain : %d, Exposure (Absolute) : %d\n", m_cap_ocams->get_control("Gain"), m_cap_ocams->get_control("Exposure (Absolute)"));
        printf("---------------------------------------------------------------\n");
        if(!m_cap_ocams->start())
        {
            perror("Failed to start ocams !!");
            exit(0);
        }
        m_size.width = camFormat.width;        m_size.height = camFormat.height;            
        m_image_size = camFormat.image_size;
        m_mat_src_ocams.create(m_size, CV_8UC2);  
    }
    else
    {
        m_cap_ocv = new cv::VideoCapture(idx_cam);
        //if(!m_cap_ocv.open(idx_cam))
        if(!m_cap_ocv)
        {
            //std::cout << "Can NOT open the camera : " << id_cam << " as " << idx_cam << " th camera !!" << std::endl;
            std::cout << "Can NOT open the camera as " << idx_cam << " th camera !!" << std::endl;
            exit(0);
        }
        set_properties(saiz, fps);
        m_cap_ocv->grab(); 
        m_size.width = m_cap_ocv->get(CV_CAP_PROP_FRAME_WIDTH);
        m_size.height = m_cap_ocv->get(CV_CAP_PROP_FRAME_HEIGHT);
    }
    int skale = is_ocams() ? 1 : 2; 
    if (saiz.width * skale != m_size.width || saiz.height != m_size.height)
    {
        std::cout << "Current camera size : " << m_size << std::endl;
        std::cout << "Requested camera size : " << saiz << std::endl;
        std::cout << "Can NOT set the resolution : [" << saiz.width * skale << " x " << saiz.height << "]" << std::endl;
        exit(0);
    } 
}


StereoCam::~StereoCam()
{
    if(m_cap_ocams)
    {
        if(m_cap_ocams->is_running())
        {
            m_cap_ocams->stop();
        }
        delete m_cap_ocams;
    }
    if(m_cap_ocv)
    {
        m_cap_ocv->release();
        delete m_cap_ocv;
    }
}

bool StereoCam::is_opened()
{
    return is_ocams() ? m_cap_ocams->is_running() : m_cap_ocv->isOpened();
}

bool StereoCam::set_properties(cv::Size& saiz, int fps)
{
    if(is_ocams())
    {        
        m_cap_ocams->set_format(saiz.width, saiz.height, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, fps);
    }
    else
    {
/*
		m_cap_ocv->grab();
        // Set the video resolution (2*Width * Height)$
        m_cap_ocv->set(CV_CAP_PROP_FRAME_WIDTH, saiz.width * 2);
        m_cap_ocv->set(CV_CAP_PROP_FRAME_HEIGHT, saiz.height);
		m_cap_ocv->set(CV_CAP_PROP_FPS, fps);
*/
		set_cam_properties_ocv(m_cap_ocv, saiz.width * 2, saiz.height, fps);
    }
    return check_if_request_is_accepted(saiz, fps);
}

bool StereoCam::read_left_right(cv::Mat& mat_full, cv::Mat& mat_l, cv::Mat& mat_r)
{
    if(is_ocams())
    {
        int knt = 0, size = m_cap_ocams->get_frame(m_mat_src_ocams.data, m_image_size);
        while(-1 == size && knt < 100)
        {
            printf("Can NOT get image from camera. Error number : %d\n", errno); 
            m_cap_ocams->stop();
            m_cap_ocams->start();
            size = m_cap_ocams->get_frame(m_mat_src_ocams.data, m_image_size);
            knt++;
        }
        if (-1 == size) return false;
        cv::split(m_mat_src_ocams, m_mat_dst_ocams);
        cv::cvtColor(m_mat_dst_ocams[0], mat_r, CV_BayerGB2BGR);   
        cv::cvtColor(m_mat_dst_ocams[1], mat_l, CV_BayerGB2BGR);   
        merge_mat_side_by_side(mat_full, mat_l, mat_r);

    }
    else
    {
        bool is_captured = m_cap_ocv->read(mat_full);
        if(!is_captured) return false;
        int w_half = mat_full.cols / 2;
        mat_l = mat_full(cv::Rect(0, 0, w_half, mat_full.rows));
        mat_r = mat_full(cv::Rect(w_half, 0, w_half, mat_full.rows));
        //mat_full = m_mat_ocv;
    }
    //imshow("mat_full", mat_full);   imshow("mat_l", mat_l); imshow("mat_r", mat_r); cv::waitKey();
    return true;
}


bool StereoCam::check_if_request_is_accepted(const cv::Size& saiz, const int fps)
{
    compute_camera_resolution_and_fps();
	bool is_accepted =
		saiz.width ==  m_size.width && 
		saiz.height == m_size.height && 
		fps == m_frame_rate;
	return is_accepted;
}

cv::Size StereoCam::get_camera_resolution()
{
	compute_camera_resolution_and_fps();
	return m_size;
}

void StereoCam::compute_camera_resolution_and_fps()
{
    if(is_ocams())
    {  
		Withrobot::camera_format fmt;     
        m_cap_ocams->get_current_format(fmt);
        m_size.width = fmt.width;  m_size.height = fmt.height;	m_frame_rate = fmt.frame_rate;
    }
    else
    {
		/*
        int w = (int)m_cap_ocv->get(CV_CAP_PROP_FRAME_WIDTH), 
            h = (int)m_cap_ocv->get(CV_CAP_PROP_FRAME_HEIGHT),
            frame_rate = (int)m_cap_ocv->get(CV_CAP_PROP_FPS);
        m_size.width = w;  m_size.height = h;	m_frame_rate = frame_rate;
		*/
		get_cam_properties_ocv(m_size.width, m_size.height, m_frame_rate, m_cap_ocv); 
    }
}

bool StereoCam::is_ocams()
{
    return m_is_ocams;
}

