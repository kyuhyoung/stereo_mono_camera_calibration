/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warranty, support or any guarantee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OPENCV WEBSITES:
     Homepage:      http://opencv.org
     Online docs:   http://docs.opencv.org
     Q&A forum:     http://answers.opencv.org
     Issue tracker: http://code.opencv.org
     GitHub:        https://github.com/opencv/opencv/
   ************************************************** */

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <experimental/filesystem>

#include "util.h"
#include "StereoCam.h"
//namespace fs = std::experimental::filesystem;


using namespace cv;
using namespace std;


/*
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



static int print_help()
{
    cout <<
            " Given a list of chessboard images, the number of corners (nx, ny)\n"
            " on the chessboards, and a flag: useCalibrated for \n"
            "   calibrated (0) or\n"
            "   uncalibrated \n"
            "     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
            "         matrix separately) stereo. \n"
            " Calibrate the cameras and display the\n"
            " rectified results along with the computed disparity images.   \n" << endl;
    cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list XML/YML file default=../data/stereo_calib.xml>\n" << endl;
    return 0;
}

/*
string python_join_equivalent(const string& dir_to_file, const string& filename)
{
    std::experimental::filesystem::path dir(dir_to_file);
    std::experimental::filesystem::path fn(filename);
    std::experimental::filesystem::path full_path = dir / fn;
    return full_path.u8string();
}
*/

void draw_count(Mat& img, bool is_left, int kount, int n_max, bool is_in_skipping_list)
{
    char text[200];
    sprintf(text, "%s of %d/%d", is_left ? "left" : "right", kount, n_max);
    if(is_in_skipping_list)
    {
        sprintf(text, "%s. %s", text, "This image skipped");
    }
    Point textOrg(img.cols / 50, img.rows / 8);
    int fontFace = FONT_HERSHEY_DUPLEX;
    double fontScale = 1.2;
    int thickness = 1;
    putText(img, text, textOrg, fontFace, fontScale, Scalar(255, 255, 255), thickness, 8);
    return;
}


void get_rectification_map_mono(Mat rmap[], /*Rect& validRoi,*/ const string& yml_mono_int_ext, const Size& imageSize, const bool& useFisheye)
{
    //bool isVerticalStereo;
    cv::FileStorage fs(yml_mono_int_ext, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", yml_mono_int_ext.c_str());
        exit(0);
    }
    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix; 
    fs["distortion_coefficients"] >> distCoeffs;

	//Mat view, rview, map1, map2;
	if (useFisheye)
	{
		Mat newCamMat;
		fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize, Matx33d::eye(), newCamMat, 1);
		fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize, CV_16SC2, rmap[0], rmap[1]);
	}
	else
	{
		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, rmap[0], rmap[1]);
	}
	return;
}





//bool get_rectification_map(Mat *cameraMatrix, Mat *distCoeffs, Mat& R, Mat& T, Mat rmap[][2], const string& yml_stereo_int, const string& yml_stereo_ext)
bool get_rectification_map_stereo(Mat rmap[][2], Rect validRoi[], const string& yml_stereo_int, const string& yml_stereo_ext, Size& imageSize, double alfa)
{
    bool isVerticalStereo;
    cv::FileStorage fs(yml_stereo_int, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", yml_stereo_int.c_str());
        exit(0);
    }
    Mat cameraMatrix[2], distCoeffs[2];
    fs["M1"] >> cameraMatrix[0];    fs["M2"] >> cameraMatrix[1];
    fs["D1"] >> distCoeffs[0];      fs["D2"] >> distCoeffs[1];
    fs.open(yml_stereo_ext, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", yml_stereo_ext.c_str());
        exit(0);
    }
    Mat R, T;
    fs["R"] >> R;                   fs["T"] >> T;
    Mat R1, R2, P1, P2, Q;
    //stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, alfa, imageSize, &validRoi[0], &validRoi[1]);
    isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    return isVerticalStereo;
}

/*
//bool retrieve_left_and_right_image(Mat *li_img, VideoCapture& cap, bool is_gray)
bool retrieve_left_and_right_image(Mat& img_l, Mat& img_r, VideoCapture& cap, bool is_gray)
{
    Mat frame;
    cap >> frame;
    if (is_gray) cvtColor(frame, frame, CV_RGB2GRAY); 
    //li_img[0] = frame(Rect(0, 0, frame.cols / 2, frame.rows));
    img_l = frame(Rect(0, 0, frame.cols / 2, frame.rows));
    //li_img[1] = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
    img_r = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
    return true;
}
*/

Size get_image_size(StereoCam *cap_stereo, VideoCapture *cap_mono, const string& filename, const string& dir_img)
{
	bool use_cam = cap_stereo || cap_mono;
	if (use_cam)	
	{
		if(cap_stereo)
		{
			cv::Size size_camera = cap_stereo->get_camera_resolution();			
			return Size(size_camera.width / (cap_stereo->is_ocams() ? 1 : 2), size_camera.height);
		}
		else
		{
			return Size(cap_mono->get(CV_CAP_PROP_FRAME_WIDTH), cap_mono->get(CV_CAP_PROP_FRAME_HEIGHT));
		}
    }
    else 
    {
        std::string path_img = python_join_equivalent(dir_img, filename);
        cout << "path_img : " << path_img << endl;
        Mat img = imread(path_img, 0);
        if(img.empty())
        {
            cout << "Can not read image file : " << path_img << endl;
            exit(0);
        }
        return img.size();
    }
}    

bool enough_time_has_passed(int sec_int, double sec_saved_last)
{
    double sec_elaps = what_time_in_seconds_is_it_now() - sec_saved_last;
    return sec_elaps > (double)sec_int;
}



static void
rectify(StereoCam *cap_stereo, VideoCapture *cap_mono, const bool& is_mono, const vector<string>& imagelist, const string& dir_image, const string& dir_rectified, const string& yml_mono_int_ext, const string& yml_stereo_int, const string& yml_stereo_ext, const string& postphix, const int& sec_disp, const double& alfa)
{
	bool use_cam = cap_stereo || cap_mono;
    int nimages, n_list = is_mono ? 1 : 2;
    if (use_cam)
    {
        nimages = INT_MAX;
    }
    else
    { 
        if((!is_mono) && (imagelist.size() % n_list != 0))
        {
            cout << "Error: the image list contains odd (non-even) number of elements even though it is stereo\n";
            return;
        }
        nimages = imagelist.size() / n_list;
    }

    //const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    //vector<vector<Point2f> > imagePoints[2];
    //vector<vector<Point3f> > objectPoints;

    int iI, iF = 1, iLR, w_r, h_r;
    double sf, sec_saved_last;
    char buff[200];
    //imagePoints[0].resize(nimages);
    //imagePoints[1].resize(nimages);
    //vector<string> goodImageList;
    stringstream ss;
    Mat cameraMatrix[2], distCoeffs[2], R, T, canvas, rmapp[2][2], li_img[2], img_full;
    Rect validRoi[2];
    Size imageSize = get_image_size(cap_stereo, cap_mono, imagelist[0], dir_image);
    cout << "imageSize : " << imageSize << endl;
    bool isVerticalStereo = false;
	if (is_mono)
	{
		get_rectification_map_mono(rmapp[0], /*validRoi[0],*/ yml_mono_int_ext, imageSize, alfa);
	}
	else 
	{
		isVerticalStereo = get_rectification_map_stereo(rmapp, validRoi, yml_stereo_int, yml_stereo_ext, imageSize, alfa);
    	cout << "isVerticalStereo : " << isVerticalStereo << endl;
    	cout << "validRoi[0] : " << validRoi[0] << endl;
	}
    if( !isVerticalStereo )
    {
    	sf = is_mono ? 1 : 600./MAX(imageSize.width, imageSize.height);
    	w_r = cvRound(imageSize.width * sf);
    	h_r = cvRound(imageSize.height * sf);
    	canvas.create(h_r, w_r * 2, CV_8UC3);
    	//canvas.create(imageSize.height, imageSize.width, CV_8UC3);
    }
    else
    {
       	sf = 300./MAX(imageSize.width, imageSize.height);
		w_r = cvRound(imageSize.width * sf);
       	h_r = cvRound(imageSize.height * sf);
       	canvas.create(h_r * 2, w_r, CV_8UC3);
       	//canvas.create(imageSize.height * 2, imageSize.width, CV_8UC3);
    }
	
    sec_saved_last = use_cam ? what_time_in_seconds_is_it_now() : -1;
    cout << "nimages : " << nimages << endl;
    for(iI = 0; iI < nimages; iI++)
    {
        if (use_cam)
        {	
			if(cap_stereo)
			{
				if(!cap_stereo->read_left_right(img_full, li_img[0], li_img[1]))
				{
					cout << "something wrong !!" << endl;					exit(0);
				}
			}
			else
			{
				//if(!retrieve_left_and_right_images_ocv(li_img[0], li_img[1], *kap, false))
				if(!cap_mono->read(li_img[0]))
				{
					cout << "something wrong !!" << endl;					exit(0);
				}
            	//cout << "li_img[0].size() : " << li_img[0].size() << endl;    
			}
        }
        else 
        {
            for(iLR = 0; iLR < n_list; iLR++)
            {
                const string& filename = imagelist[iI * n_list + iLR];
                string path_img = python_join_equivalent(dir_image, filename);
                cout << "path_img : " << path_img << endl;
                li_img[iLR] = imread(path_img, CV_LOAD_IMAGE_COLOR);
                if(li_img[iLR].empty())
                {
                    cout << "Can not read image file : " << path_img << endl;
                    exit(0);
                }
                if( li_img[iLR].size() != imageSize )
                {
                    cout << "The image " << path_img << " has the size different from the first image size. Skipping the pair\n";
                    exit(0);
                }
            }
            if (n_list > iLR)
            {
                cout << "something wrong !!" << endl;
                exit(0);
            }
        }

        for(iLR = 0; iLR < n_list; iLR++)
        {
            Mat cimg;
            remap(li_img[iLR], cimg, rmapp[iLR][0], rmapp[iLR][1], INTER_LINEAR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w_r * iLR, 0, w_r, h_r)) : canvas(Rect(0, h_r * iLR, w_r, h_r));
			if(is_mono)
			{
				cimg.copyTo(canvasPart);
			}
			else
			{
            	resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            	Rect vroi(cvRound(validRoi[iLR].x * sf), cvRound(validRoi[iLR].y * sf),
                	cvRound(validRoi[iLR].width * sf), cvRound(validRoi[iLR].height * sf));
            	rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
			}
        }
		if(is_mono)
		{
			Mat canvasPart = canvas(Rect(w_r, 0, w_r, h_r));
			li_img[0].copyTo(canvasPart);
		}
		else
		{
        	if( !isVerticalStereo )
            	for(int j = 0; j < canvas.rows; j += 16 )
                	line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        	else
            	for(int j = 0; j < canvas.cols; j += 16 )
                	line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		}

		cout << endl << "canvas.size() : " << canvas.size() << endl;

        if (use_cam)
        {
            imshow("rectified", canvas);
            bool shall_save = enough_time_has_passed(sec_disp, sec_saved_last);
            if (shall_save)
            {
                sprintf(buff, "_%02d", iF++);
                std::string full_path, buffAsStdStr = "rectified_" + postphix + buff;
                full_path = python_join_equivalent(dir_rectified, buffAsStdStr + ".png");
                imwrite(full_path, canvas);
                cout << full_path << " is just saved" << endl;
                sec_saved_last = what_time_in_seconds_is_it_now();
            }
            char c = (char)waitKey(1);
            if( c == 27 || c == 'q' || c == 'Q' ) break;
        }
        else
        {
            sprintf(buff, "_%02d", iI + 1);
            std::string full_path, buffAsStdStr = "rectified_" + postphix + buff;
            //full_path = python_join_equivalent(dir_img, buffAsStdStr + "_" + postphix + ".png");
            full_path = python_join_equivalent(dir_rectified, buffAsStdStr + ".png");
            //imshow(buffAsStdStr, canvas);
            imshow(full_path, canvas);
            imwrite(full_path, canvas);
            cout << full_path << " is just saved" << endl;
            //imshow("rectified", canvas);
            char c = (char)waitKey(sec_disp * 1000);
            if( c == 27 || c == 'q' || c == 'Q' ) break;
            destroyWindow(full_path);
        }

    }
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

vector<int> get_one_based_skipping_indices(string& one_based_indices_to_skip)
{
    vector<int> vect;
    std::stringstream ss(one_based_indices_to_skip);
    int i;
    while (ss >> i)
    {
        vect.push_back(i);
        if (ss.peek() == ',')
            ss.ignore();
    }

    for (size_t i = 0; i < vect.size(); i++)
        std::cout << vect.at(i) << std::endl;


    return vect;

}


int main(int argc, char** argv)
{
    double alfa;
    bool is_mono, use_cam = false;
    VideoCapture *cap_mono = NULL;
    StereoCam *cap_stereo = NULL;
    int sec_disp;//, width = -1, height = -1, cam_idx = -1;
    vector<int> li_one_based_indices_to_skip;
    //Size boardSize;
    string /*one_based_indices_to_skip,*/ postphix, yml_stereo_int, yml_stereo_ext, yml_mono_int_ext, dir_img, dir_rect, imagelistfn;
    //bool showRectified;
    //cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{s|1.0|}{nr||}{help||}{@input|../data/stereo_calib.xml|}");
    cv::CommandLineParser parser(argc, argv, "{calib||the path to the yml file of mono camera calibration parametes}      {dir_rect||directory where the resulting rectified images will be saved.}      {alfa|1.0|alpha value for rectification}      {sec|7|seconds for displaying rectified image}      {post||postfix of rectified imagest to save}      {width||camera image width}      {height||camera image height}      {int||the path to the stereo calibration intrinsic parameters}      {ext||the path to the stereo calibration extrinsic parameters}      {mono||If the camera is monocular}      {fps|30|frame rate of camera}      {ocam|0|if non-zero, the camera is ocams. If zero, it is not ocams}      {cam||camera index}      {nr||short for no rectification}      {help||}      {dir_img||the path to the left and right image pairs}      {input||path to xml file of input image list}");

    if (parser.has("help"))
        return print_help();
    //showRectified = !parser.has("nr");
    if(!(parser.has("post")))
    {
        cout << "Please provide -post= which is the postfix of the file name of rectified images to be saved !!" << endl;
        exit(0);
    }
	is_mono = parser.has("mono");
    alfa = parser.get<double>("alfa");
    sec_disp = parser.get<int>("sec");
    postphix = parser.get<string>("post");
    cout << "postphix : " << postphix << endl;
    if((!is_mono) && !(parser.has("int") && parser.has("ext")))
    {
        cout << "You should provide -int= and -ext= , which are the yml files of stereo camera intrinsic and extrinsic parameters" << endl;
        exit(0);
    }
	if (parser.has("cam"))
	{
		if(!parser.has("ocam"))
        {            
            cout << "You should give -ocam= which is non-zero for withrobot ocams camera !!" << endl;
            exit(0);
        }
        if(!(parser.has("width") && parser.has("height")))
        {
            cout << "You should give -width= and -height, which are the camera resolution !!" << endl;
            exit(0);
        }
        cv::Size2i size_cam(parser.get<int>("width"), parser.get<int>("height"));
        int cam_idx = parser.get<int>("cam"), fps = parser.get<int>("fps");
		bool is_ocam = 0 != parser.get<int>("ocam");//, is_mono = parser.has("mono");
		if (is_mono)
		{
			cap_mono = new VideoCapture(cam_idx);
			//if(!cap.open(cam_idx))
			if(!cap_mono)
			{
				cout << "Can NOT open camera : " << cam_idx << endl;
				exit(0);
			}
			if(!cap_mono->isOpened())
			{
				cout << "Can NOT open camera : " << cam_idx << endl;
				exit(0);
			}
			cap_mono->grab();
			cap_mono->set(CV_CAP_PROP_FRAME_WIDTH, size_cam.width);
			cap_mono->set(CV_CAP_PROP_FRAME_HEIGHT, size_cam.height);
			cap_mono->grab();
		}
		else
		{
			//kam.create(is_ocam, cam_idx, size_cam, fps); 
			cap_stereo = new StereoCam(is_ocam, cam_idx, size_cam, fps); 
		}
		use_cam = true;
    }
    if (!use_cam)
    {
        //if(!(parser.has("input") && parser.has("dir") && parser.has("int") && parser.has("ext")))
        if(!(parser.has("input") && parser.has("dir_img")))
        {
            cout << "Since you did not give -cam=(camera index), you should provide -input= and -dir_img= !!" << endl;
            exit(0);
        }
        imagelistfn = parser.get<string>("input");
        cout << "imagelistfn : " << imagelistfn << endl;
        dir_img = parser.get<string>("dir_img");
        cout << "dir_img : " << dir_img << endl;
    }
    dir_rect = parser.get<string>("dir_rect");
    cout << "dir_rect : " << dir_rect << endl;
    if(!mkdir_if_not_exist(dir_rect.c_str()))
    {
        exit(0);
    }
	
	if(is_mono)
	{
		yml_mono_int_ext = parser.get<string>("calib");
	}
	else
	{
    	yml_stereo_int = parser.get<string>("int");
    	cout << "yml_stereo_int : " << yml_stereo_int << endl;
    	yml_stereo_ext = parser.get<string>("ext");
    	cout << "yml_stereo_ext : " << yml_stereo_ext << endl;
	} 
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    vector<string> imagelist;
    if (!use_cam)
    { 
        bool ok = readStringList(imagelistfn, imagelist);
        if(!ok || imagelist.empty())
        {
            cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
            return print_help();
        }
    }

    //StereoCalib(imagelist, dir_img, boardSize, squareSize, false, true, showRectified);
    rectify(cap_stereo, cap_mono, is_mono, imagelist, dir_img, dir_rect, yml_mono_int_ext, yml_stereo_int, yml_stereo_ext, postphix, sec_disp, alfa);
    //rectify(use_cam, imagelist, dir_img, e, squareSize, li_one_based_indices_to_skip, true, true, showRectified);
	if(cap_stereo) delete cap_stereo;
	if(cap_mono) delete cap_mono;
    return 0;
}
