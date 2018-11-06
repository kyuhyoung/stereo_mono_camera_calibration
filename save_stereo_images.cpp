///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*****************************************************************************************
 ** This sample demonstrates how to capture stereo images and calibration parameters    **
 ** from the ZED camera with OpenCV without using the ZED SDK.                          **
 *****************************************************************************************/

// General includes
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
//#include <sys/stat.h>


// OpenCV includes
#include <opencv2/opencv.hpp>

// Sample includes
#include "calibration.hpp"

#include "StereoCam.h"
#include "util.h"

#include <vector>
// Namespaces
using namespace cv;
using namespace std;

#define SEC_INTERVAL        2

typedef struct
{

	StereoCam *cap_2;
	cv::VideoCapture *cap_1;

}	struct_fetch;

typedef struct
{
	bool is_mono;
    int sec_detect_interval;
    float th_ratio_overlap;
    cv::Size boardSize;
    cv::Size2i imageSize;
    string dir_img;
    vector<string> imagelist;
    

} struct_save;






bool save_done = false;
cv::Mat g_mat_full, g_mat_l, g_mat_r;
pthread_mutex_t mutex_lock_img;

RNG rng(12345);
//#define N_FRM       10

/*
#include <experimental/filesystem>
string python_join_equivalent(const string& dir_to_file, const string& filename)
{
  std::experimental::filesystem::path dir(dir_to_file);
  std::experimental::filesystem::path fn(filename);    
  std::experimental::filesystem::path full_path = dir / fn;
  return full_path.u8string();                                                                               
}

//------------ Get the current time in the unit of second --------------  
//  original code : https://github.com/pjreddie/darknet/blob/master/src/utils.c
#include <sys/time.h>
#include <unistd.h>
double what_time_in_seconds_is_it_now()                                                                       
{                                   
    struct timeval time;
    if (gettimeofday(&time,NULL)) 
    {
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}
*/

double getMSE(const Mat& I1, const Mat& I2)
{
    Mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    Scalar s = sum(s1);         // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double  mse =sse /(double)(I1.channels() * I1.total());
        return mse;
        // Instead of returning MSE, the tutorial code returned PSNR (below).
        //double psnr = 10.0*log10((255*255)/mse);
        //return psnr;
    }
}





//bool is_camera_shaking(vector<Mat> li_img, int iCur, int len, double thres)
bool is_camera_shaking(Mat* li_img, int iCur, int len, double thres)
{
    int iPre = (iCur + 1) % len;
    double mse = getMSE(li_img[iCur], li_img[iPre]);
    return mse > thres;
 }

static int print_help()
{

    return 0;
}



static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}


bool check_if_chessboard_in_the_image(vector<Point>& foundPoints_left, vector<Point>& foundPoints_right, Mat& left_raw, Mat& right_raw, Size& chessboardDimensions)
{
    bool is_mono = right_raw.empty(), found_left = findChessboardCorners(left_raw, chessboardDimensions, foundPoints_left, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK); 
	if(is_mono)
	{
		return found_left;
	}
	else
	{
		if(found_left)
		{
			bool found_right = findChessboardCorners(right_raw, chessboardDimensions, foundPoints_right, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
			return found_right;
		}
		return false;
	}
}

Mat get_overlap_mask(Mat& mask1, Mat& mask2)
{
    Mat overlap;
    bitwise_and(mask1, mask2, overlap);
    return overlap;
}

int compute_mask_area(Mat& mask)
{
    CV_Assert(mask.type() == CV_8UC1 );
    return countNonZero(mask);
}

Mat make_chessboard_mask(const vector<Point>& li_corner, const Size& imageSize)
{
    vector<Point> li_corner_int;
    for (size_t i = 0; i < li_corner.size(); i++)
    {
        li_corner_int.push_back(Point(cvRound(li_corner[i].x), cvRound(li_corner[i].y)));
    }
    Mat mask = Mat::zeros(imageSize, CV_8UC1);
    
    fillConvexPoly(mask, li_corner_int, Scalar(1), 1, 0);
    return mask;             
}



bool check_if_two_boards_ard_overlapped_enough(Mat& mask1, Mat& mask2, int area1, int area2, float th_ratio_overlap)
{
    Mat mask_overlap = get_overlap_mask(mask1, mask2);
    int area_overlap = compute_mask_area(mask_overlap);
    float ratio1 = (float)area_overlap / (float)area1, ratio2 = (float)area_overlap / (float)area2;
    return ratio1 > th_ratio_overlap && ratio2 > th_ratio_overlap;
}




int check_if_chessboard_is_duplicated(vector<vector<Point>>& li_li_corner_l, vector<vector<Point>>& li_li_corner_r, vector<Mat>& li_chessboard_mask_l, vector<Mat>& li_chessboard_mask_r, vector<int>& li_area_mask_l, vector<int>& li_area_mask_r, const vector<Point>& li_corner_l, const vector<Point>& li_corner_r, const float& th_ratio_overlap, const Size& imageSize) 
{
	bool is_mono = li_corner_r.empty(); 
	Mat chessboard_mask_l = make_chessboard_mask(li_corner_l, imageSize), chessboard_mask_r;
	int iS, area_mask_l = compute_mask_area(chessboard_mask_l), area_mask_r = -1, n_saved  = li_chessboard_mask_l.size();
	if(!is_mono)
	{
		chessboard_mask_r = make_chessboard_mask(li_corner_r, imageSize);
		area_mask_r = compute_mask_area(chessboard_mask_r);
	}
	for(iS = 0; iS < n_saved; iS++)
	{
		if (check_if_two_boards_ard_overlapped_enough(li_chessboard_mask_l[iS], chessboard_mask_l, li_area_mask_l[iS], area_mask_l, th_ratio_overlap))
		{
		       //return true;
			return iS;
		}    
		if(!is_mono)
		{
			if (check_if_two_boards_ard_overlapped_enough(li_chessboard_mask_r[iS], chessboard_mask_r, li_area_mask_r[iS], area_mask_r, th_ratio_overlap))
			{
            //return true;
				return iS;
			}
		}
	}
	li_li_corner_l.push_back(li_corner_l);             
	li_chessboard_mask_l.push_back(chessboard_mask_l);  
	li_area_mask_l.push_back(area_mask_l);              
	if(!is_mono)
	{
		li_li_corner_r.push_back(li_corner_r);
		li_chessboard_mask_r.push_back(chessboard_mask_r);
		li_area_mask_r.push_back(area_mask_r);
	}
    //return false;
    return -1;
}



void draw_chessboard_masks_both(Mat& img, vector<vector<Point>> &li_li_hull_l,vector<vector<Point>> &li_li_hull_r, vector<Scalar>& li_color, int idx_duplicated) 
{
    
    //cout << "li_li_hull_l.size() : " << li_li_hull_l.size() << endl;
    int thickness, iH, n_saved = li_li_hull_l.size(); 
    for(iH = 0; iH < n_saved; iH++)
    {   
        thickness = iH == idx_duplicated ? -1 : 2;
        //cout << "li_li_hull_l[iH].size() : " << li_li_hull_l[iH].size(); 
        drawContours(img, li_li_hull_l, iH, li_color[iH], thickness);
		if(!li_li_hull_r.empty())
		{
        	drawContours(img, li_li_hull_r, iH, li_color[iH], thickness, 8, Mat(), INT_MAX, Point(img.cols / 2, 0));
		}
	}
    return;
}



void draw_chessboard_masks(Mat& img, vector<vector<Point>> &li_li_corner_hull, vector<Scalar>& li_color)
{
    cout << "li_li_corner_hull.size() : " << li_li_corner_hull.size() << endl;
    for(size_t iH = 0; iH < li_li_corner_hull.size(); iH++)
    {
        cout << "li_li_corner_hull[iH].size() : " << li_li_corner_hull[iH].size(); 
        drawContours(img, li_li_corner_hull, iH, li_color[iH], 2);
    }
    return;
}

void draw_count(Mat& img, int kount, int n_max)
{
    char text[200];
    sprintf(text,"%d / %d", kount, n_max);

    Point textOrg(img.cols / 50, img.rows / 8);
    //Point textOrg(img.rows / 90, img.cols / 11);
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 1.2;
    int thickness = 2;

    putText(img, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    return;

}


vector<Scalar> make_color_list(int n_color = 0)
{
    vector<Scalar> li_color;
    li_color.push_back(Scalar(255, 0, 0));
    li_color.push_back(Scalar(0, 255, 0));
    li_color.push_back(Scalar(0, 0, 255));
    li_color.push_back(Scalar(255, 255, 0));
    li_color.push_back(Scalar(0, 255, 255));
    li_color.push_back(Scalar(255, 0, 255));
    li_color.push_back(Scalar(255, 128, 0));
    li_color.push_back(Scalar(0, 255, 128));
    li_color.push_back(Scalar(128, 0, 255));
    li_color.push_back(Scalar(128, 255, 0));
    li_color.push_back(Scalar(0, 128, 255));
    li_color.push_back(Scalar(255, 0, 128));
    li_color.push_back(Scalar(128, 128, 0));
    li_color.push_back(Scalar(0, 128, 128));
    li_color.push_back(Scalar(128, 0, 128));
    int iC, iStart = li_color.size();
    for(iC = iStart; iC < n_color; iC++)
    {
        li_color.push_back(Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ));
    }
    return li_color;
}

bool save_images_unless_duplicated(vector<vector<Point> >& li_li_corner_hull_l, vector<vector<Point> >& li_li_corner_hull_r, vector<Mat>& li_chessboard_mask_l, vector<Mat>& li_chessboard_mask_r, vector<int>& li_area_mask_l, vector<int>& li_area_mask_r, const int n_img_saved, const vector<string>& li_fn_img, const vector<Point>& li_corner_l, const vector<Point>& li_corner_r, const Mat& mat_l, const Mat& mat_r, const string& dir_img, const float th_ratio_overlap, const Size2i& imageSize)
{
	bool is_mono = li_corner_r.empty();
	vector<Point> hull_l, hull_r;
	convexHull(li_corner_l, hull_l); 
	//if(!(param_save->is_mono))
	if(!is_mono)
	{
		convexHull(Mat(li_corner_r), hull_r);
	}
    int idx_duplicated = check_if_chessboard_is_duplicated(li_li_corner_hull_l, li_li_corner_hull_r, li_chessboard_mask_l, li_chessboard_mask_r, li_area_mask_l, li_area_mask_r, hull_l, hull_r, th_ratio_overlap, imageSize);
	if (idx_duplicated < 0)
	{
		//save_images(param_save->imagelist, n_img_saved * (is_mono ? 1 : 2), is_mono ? mat_full_copy_2 : mat_l_copy, mat_r_copy);
		save_images(li_fn_img, n_img_saved * (is_mono ? 1 : 2), mat_l, mat_r, dir_img);
		//enough_time_has_passed = false;
		//sec_saved_last = what_time_in_seconds_is_it_now();
	}
	return true;
}
 


void pthread_join_wrap(pthread_t th, const char *name_thread)
{
    int rc, status;
    rc = pthread_join(th, (void **)&status);
    if(0 == rc)
    {
        printf("Completed join with %s. status = %d\n", name_thread, status);
    }
    else
    {
        printf("ERROR; return code from pthread_join() is %d for %s\n", rc, name_thread);
        exit(0);
    }
    return;
}

void *fetch_in_thread(void *ptr)
{
    struct_fetch *param_fetch = (struct_fetch *)ptr;
	bool is_mono = param_fetch->cap_1 != NULL; 
    //StereoCam *cap = (StereoCam *)ptr;
    cv::Mat mat_full_ori, mat_l_ori, mat_r_ori;
    int n_frm = 0;
    double fps_fetch, sec_cur, sec_pre, sec_start = what_time_in_seconds_is_it_now();//demo_time_detect = what_time_is_it_now();
    sec_pre = sec_start; 
    while (!save_done)
    {    
        n_frm++;
        //cap->read_left_right(mat_full_ori, mat_l_ori, mat_r_ori);
		if(is_mono)
		{
			param_fetch->cap_1->read(mat_full_ori);
		}
        else
		{	
			param_fetch->cap_2->read_left_right(mat_full_ori, mat_l_ori, mat_r_ori);
		}
        pthread_mutex_lock(&mutex_lock_img);
        mat_full_ori.copyTo(g_mat_full); 
		if(!is_mono)
		{
			mat_l_ori.copyTo(g_mat_l);    mat_r_ori.copyTo(g_mat_r);
		}
        pthread_mutex_unlock(&mutex_lock_img);
        sec_cur = what_time_in_seconds_is_it_now();
        if ((sec_cur - sec_pre) > SEC_INTERVAL) 
        {   
            fps_fetch = (double)(n_frm) / (sec_cur - sec_start);
            //printf("\033[2J");        printf("\033[1;1H");
            printf("\nFPS 4 fetch : %.1f\n", fps_fetch);
            sec_pre = sec_cur; 
        }
    }
    pthread_exit((void *) 0);
}


void *save_in_thread(void *ptr)
{
    struct_save *param_save = (struct_save *)ptr;
    //std::cout << "param_save->dir_img : " << param_save->dir_img << std::endl;
    //StereCam *cap = (StereoCam *)ptr;
    cv::Mat /*mat_full_ori,*/ mat_full_copy, mat_full_copy_2, mat_l_copy, mat_r_copy;
    vector<vector<Point> > li_li_corner_hull_l, li_li_corner_hull_r;
    vector<int> li_area_mask_l, li_area_mask_r;
    vector<Scalar> li_color = make_color_list();
    vector<Mat> li_chessboard_mask_l, li_chessboard_mask_r;
    int n_frm = 0, idx_duplicated = -1, n_img = param_save->imagelist.size() / (param_save->is_mono ? 1 : 2);
    bool /*is_saved,*/ chessboard_detected, enough_time_has_passed = false;
    double fps_fetch, sec_cur, sec_pre, sec_start = what_time_in_seconds_is_it_now(), sec_saved_last = what_time_in_seconds_is_it_now();
    sec_pre = sec_start; 
    while (!save_done)
    {    
        n_frm++;
        //cap->read_left_right(mat_full_ori, mat_l_ori, mat_r_ori);
        pthread_mutex_lock(&mutex_lock_img);
        g_mat_full.copyTo(mat_full_copy); 
		if(!(param_save->is_mono))
		{	
			g_mat_l.copyTo(mat_l_copy);    g_mat_r.copyTo(mat_r_copy);
		}
        pthread_mutex_unlock(&mutex_lock_img);
        if(param_save->is_mono)
		{
			mat_full_copy.copyTo(mat_full_copy_2);
		}
        //mat_full_copy.copyTo(chessboard_overlaid);
        int n_img_saved = li_li_corner_hull_l.size();
        cout << "n_img_saved : " << n_img_saved << "\t n_img : " << n_img << endl;
        //n_saved = li_li_corner_hull_l.size();
        draw_chessboard_masks_both(mat_full_copy/*chessboard_overlaid*/, li_li_corner_hull_l, li_li_corner_hull_r,li_color, idx_duplicated); 
        draw_count(mat_full_copy/*chessboard_overlaid*/, li_li_corner_hull_l.size(), n_img);
        imshow("mat_full_copy", mat_full_copy); waitKey(1);
        if (n_img_saved >= n_img) 
        {
            cout << "================================================" << endl;
            cout << "All images are saved !!!" << endl;
            cout << "================================================" << endl;
            waitKey(30000);
            save_done = true;
            break;
        }
        idx_duplicated = -1;
        if (enough_time_has_passed)
        {
            cout << "enough_time_has_passed" << endl;
            vector<Point> li_corner_l, li_corner_r;
            chessboard_detected = check_if_chessboard_in_the_image(li_corner_l, li_corner_r, param_save->is_mono ? mat_full_copy_2 : mat_l_copy, mat_r_copy, param_save->boardSize);
            if (chessboard_detected)
            {
				save_images_unless_duplicated(li_li_corner_hull_l, li_li_corner_hull_r, li_chessboard_mask_l, li_chessboard_mask_r, li_area_mask_l, li_area_mask_r, n_img_saved, param_save->imagelist, li_corner_l, li_corner_r, param_save->is_mono ? mat_full_copy_2 : mat_l_copy, mat_r_copy, param_save->dir_img, param_save->th_ratio_overlap, param_save->imageSize);
			}
        }        
        else
        {  
            double sec_now = what_time_in_seconds_is_it_now();
            double sec_elaps = sec_now - sec_saved_last;
            double sec_enough = li_li_corner_hull_l.size() ? param_save->sec_detect_interval : 2 * param_save->sec_detect_interval;
            cout << "NOT enough_time_has_passed.\tsec_elaps : " << int(sec_elaps) << " / " << sec_enough << endl;
            if (sec_elaps > sec_enough)
            {
                enough_time_has_passed = true;
            }
        }
        sec_cur = what_time_in_seconds_is_it_now();
        if ((sec_cur - sec_pre) > SEC_INTERVAL) 
        {   
            fps_fetch = (double)(n_frm) / (sec_cur - sec_start);
            //printf("\033[2J");        printf("\033[1;1H");
           printf("\nFPS 4 save : %.1f\n", fps_fetch);
            sec_pre = sec_cur; 
        }
    }
    pthread_exit((void *) 0);
}





void save_left_right_sequence_with_threads(cv::VideoCapture *cap_1, StereoCam *cap_2, struct_save& param_save)//, struct_fetch& param_fetch)
{
    srand(2222222);
    pthread_t save_thread, fetch_thread;
    //cv::Mat mat_full;
    //printf("save aaa\n");
	if(cap_2)
	{
    	if(!(cap_2->read_left_right(g_mat_full, g_mat_l, g_mat_r)))
    	{
        	//printf("save bbb\n");
        	exit(0);
    	}
	}
	else
	{
		if(!(retrieve_left_and_right_images_ocv(g_mat_l, g_mat_r, *cap_1, false)))
		{
			exit(0);
		}
		merge_mat_side_by_side(g_mat_full, g_mat_l, g_mat_r);
	}	
	struct_fetch param_fetch;	param_fetch.cap_1 = cap_1;	param_fetch.cap_2 = cap_2;
    //printf("save ccc\n");
    //std::cout << "param_save.dir_img : " << param_save.dir_img << std::endl;
    pthread_mutex_init(&mutex_lock_img, NULL);
    //pthread_mutex_init(&mutex_lock_millimeter, NULL);
    if(pthread_create(&fetch_thread, 0, fetch_in_thread, &param_fetch/*&cap*/)) error("Thread creation failed");
    if(pthread_create(&save_thread, 0, save_in_thread, &param_save)) error("Thread creation failed");
    pthread_join_wrap(fetch_thread, "fetch_thread");
    pthread_join_wrap(save_thread, "save_thread");
    return;

}







int main(int argc, char** argv) {

    string imagelistfn, str_sn;//,> dir_img;
    //Size boardSize;
    //Size2i image_size;// = cv::Size2i(1280, 720);
    cv::CommandLineParser parser(argc, argv, " {mono||If camera is monocular}	{ocam|0|If non-zero, the camera is ocams}	{fps|30|desired frame rate of camera reading} {cam|0|camera index} {show||flag for displaying}{sec_int|0|minimum interval (in seconds) between chessboard detection}{n_frm|0|# of frames for checking camera moving/shaking}       {width|0|the width of left or right image}        {height|0|the height of left or right image}        {w|0|the number of cols in the chessboard. Actually #cols + 1}        {h|0|the number of rows in the chessboard. Actually #rows + 1}        {s_mm|0.0|the physical side length (in millimeters) of a cell in the chessboard}       {th_overlap|0.0|threshold for checking two masks are overlapping too much} {th_shake|0.0|threshold for checking camera moving/shaking}                {help ?||show me the usage}        {dir_img||the path to the left and right image pairs}        {image_list|../data/stereo_calib.xml|}");

    if (parser.has("help"))
        return print_help();
	StereoCam *cap_2 = NULL;
	cv::VideoCapture *cap_1 = NULL;
    struct_save param_save;
    param_save.dir_img = parser.get<string>("dir_img");
    imagelistfn = parser.get<string>("image_list");
    if (imagelistfn.empty()) { cout << "-image_list should be given !!" << endl; return - 1; } 
    //bool is_ocam = parser.has("ocam");
    bool is_mono = parser.has("mono");
    bool is_ocam = parser.get<int>("ocam");
    param_save.sec_detect_interval = parser.get<int>("sec_int");
    if (param_save.sec_detect_interval <= 0) { cout << "-sec_int should be given !!" << endl; return - 1; } 
    int fps = parser.get<int>("fps");
    int idx_cam = parser.get<int>("cam");
    param_save.boardSize.width = parser.get<int>("w");
    if (param_save.boardSize.width <= 0) { cout << "-w, # of cols + 1 should be given !!" << endl; return - 1; } 
    param_save.boardSize.height = parser.get<int>("h");
    if (param_save.boardSize.height <= 0) { cout << "-h, # of rows + 1 should be given !!" << endl; return - 1; } 
    //printf("bbb\n"); 
    param_save.imageSize.width = parser.get<int>("width");
    if (param_save.imageSize.width <= 0) { cout << "-width, the width of left or right camera image should be given !!" << endl; return -1; }
    param_save.imageSize.height = parser.get<int>("height");
    if (param_save.imageSize.height <= 0) { cout << "-height, the height of left or right camera image should be given !!" << endl; return -1; }
    param_save.th_ratio_overlap = parser.get<float>("th_overlap");
    if (param_save.th_ratio_overlap <= 0) { cout << "-th_overlap, the threshold for checking two masks are overlap too much !!" << endl; return - 1; } 
    float squareSize = parser.get<float>("s_mm");
    if (squareSize <= 0) { cout << "-s_mm, the side length of a cell in millimeters should be given !!" << endl; return - 1; } 
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }

    if(!mkdir_if_not_exist(param_save.dir_img.c_str()))
    {
        exit(0);
    }
    bool ok = readStringList(imagelistfn, param_save.imagelist);//, is_saved;
    if(!ok || param_save.imagelist.empty())
    {
        cout << "Can NOT open " << imagelistfn << " or the string list is empty" << endl;
        return print_help();
    }
    //char key = 'r';
    //StereoCam kam(is_ocam /*id_cam,*/, idx_cam, param_save.imageSize, fps); 
	param_save.is_mono = is_mono;
    if(is_mono)
	{
		printf("main AAA\n");
		cap_1 = new cv::VideoCapture(idx_cam);
		printf("main BBB\n");
		if(!cap_1->isOpened())
		{
		printf("main EEE\n");
			return -1;
		}
		printf("main FFF\n");
		set_cam_properties_ocv(cap_1, param_save.imageSize.width, param_save.imageSize.height, fps);
		if(!check_if_request_is_accepted_ocv(cap_1, param_save.imageSize.width, param_save.imageSize.height, fps))
		{
			return -1;
		}
	}
	else
	{
		//kam(is_ocam /*id_cam,*/, idx_cam, param_save.imageSize, fps);  
		cap_2 = new StereoCam(is_ocam, idx_cam, param_save.imageSize, fps);  
		if (!(cap_2->is_opened()))
		{
			return -1;
		}
	}
    Mat chessboard_overlaid, left_raw, right_raw, full_raw/*, left_rect, right_rect*/; 
    //cout << "333" << endl;
    save_left_right_sequence_with_threads(cap_1, cap_2, param_save);
	if(cap_1) delete cap_1;
	if(cap_2) delete cap_2;
    return 0;
}
