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

//#include <sys/stat.h>

#include <experimental/filesystem>
//namespace fs = std::experimental::filesystem;

#include "util.h"

using namespace cv;
using namespace std;

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

void draw_count(Mat& img, bool is_left, int kount, int n_max, bool is_in_skipping_list, bool found)
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

    if(!found)
    {
        sprintf(text, "%s", "Corners are NOT found");
        textOrg.y = img.rows / 2;
        fontScale = 2;
        thickness = 2;
        putText(img, text, textOrg, fontFace, fontScale, Scalar(0, 0, 255), thickness, 8);
    }
 
    return;
}

void draw_grids(/*InputOutputArray*/ Mat& cimg, Size& boardSize, InputArray /*vector<Point2f>&*/ li_corner, int radius)
{
	Mat corners = li_corner.getMat();
	const Point2f* corners_data = corners.ptr<Point2f>(0);
	const int line_max = 7;
    const int shift = 0;
	//const int radius = 4;
	const int r = radius * (1 << shift);
	int line_type = LINE_AA;
	static const int line_colors[line_max][4] =
	{
		{0,0,255,0},
		{0,128,255,0},
		{0,200,200,0},
		{0,255,0,0},
		{200,200,0,0},
		{255,0,0,0},
		{255,0,255,0}
	};

	//printf("grids aa\n");
	//cv::Point2i prev_pt;
	for (int y = 0, i = 0; y < boardSize.height; y++)
	{
		const int* line_color = &line_colors[y % line_max][0];
		Scalar color(line_color[0], line_color[1], line_color[2], line_color[3]);
		//if (cn == 1) color = Scalar::all(200);
		//color *= scale;
		for (int x = 0; x < boardSize.width; x++, i++)
		{
			cv::Point2i pt(
				cvRound(corners_data[i].x*(1 << shift)),
				cvRound(corners_data[i].y*(1 << shift))
				);
			//	
			//if (i != 0) line(cimg, prev_pt, pt, color, 1, line_type, shift);
			line(cimg, Point(pt.x - r, pt.y - r), Point( pt.x + r, pt.y + r), color, 1, line_type, shift);
			line(cimg, Point(pt.x - r, pt.y + r), Point( pt.x + r, pt.y - r), color, 1, line_type, shift);
			//circle(cimg, pt, r + (1 << shift), color, 1, line_type, shift);
			//prev_pt = pt;
			//drawMarker(cimg, pt, color, marker_type, marker_size, marker_thickness);
				
		}
	}

	//printf("grids bb\n");
    return;
}

void draw_chessboard_grids(Mat& img, bool found, bool is_in_skipping_list, bool is_left, int kount, int n_img, int radius, /*int marker_type, int marker_size, int marker_thickness,*/ Size& boardSize, vector<Point2f>& li_corner_init, vector<Point2f>& li_corner_subpxl)
{
    
    //cout << filename << endl;
    Mat cimg, cimg1;
    cvtColor(img, cimg, COLOR_GRAY2BGR);
    if(found)
    {
        drawChessboardCorners(cimg, boardSize, li_corner_subpxl, found);
        draw_grids(cimg, boardSize, li_corner_init, radius /*marker_type, marker_size, marker_thickness*/);
    }
    else
    {
        drawChessboardCorners(cimg, boardSize, li_corner_init, found);    
    }    
    //draw_count(cimg, 0 == k, i + 1, nimages, is_in_skipping_list, found);
	//printf("chess aa\n");
    draw_count(cimg, is_left, kount, n_img, is_in_skipping_list, found);
	//printf("chess bb\n");
    double sf = 640./MAX(img.rows, img.cols);
    //resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
    resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR);
    imshow("corners", cimg1);
    //char c = (char)waitKey(500);
    char c = (char)waitKey(found ? 3000 : 9000);
	//printf("draw aa\n");
    //char c = (char)waitKey(1);
	//printf("draw bb\n");
    //char c = (char)waitKey();
    if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
        exit(-1);
    
    return;
}



 //! [compute_errors]    
 static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat& cameraMatrix , const Mat& distCoeffs, vector<float>& perViewErrors, bool fisheye)
{
	vector<Point2f> imagePoints2;     
	size_t totalPoints = 0;   
	double totalErr = 0, err;     
	perViewErrors.resize(objectPoints.size());       
	for(size_t i = 0; i < objectPoints.size(); ++i )   
	{
		if (fisheye)   
		{ 
			fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,      
			 distCoeffs);     
		} 
		else
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);   
		}
		err = norm(imagePoints[i], imagePoints2, NORM_L2);   
		size_t n = objectPoints[i].size();   
		perViewErrors[i] = (float) std::sqrt(err*err/n);
		totalErr        += err*err;  
		totalPoints     += n;   
	}
	return std::sqrt(totalErr/totalPoints);     
 }
 //! [compute_errors]   












static bool runCalibration_mono(Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs,  vector<float>& reprojErrs, double& totalAvgErr, const Size& imageSize, const vector<vector<Point2f> > &imagePoints, const vector<vector<Point3f> > objectPoints, const int& flag, const bool& useFisheye)//, newObjPoints, grid_width, release_object)
{
	//! [fixed_aspect]
	cameraMatrix = Mat::eye(3, 3, CV_64F);
/*	
	if(flag & CALIB_FIX_ASPECT_RATIO )  
		cameraMatrix.at<double>(0,0) = s.aspectRatio; 
	//! [fixed_aspect]     
*/	
	if (useFisheye) 
	{ 
		distCoeffs = Mat::zeros(4, 1, CV_64F); 
	} 
	else 
	{
		distCoeffs = Mat::zeros(8, 1, CV_64F);  
	}
	
/*
	vector<vector<Point3f> > objectPoints(1);   
	calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints.resize(imagePoints.size(),objectPoints[0]);   
*/	
	//Find intrinsic and extrinsic camera parameters           
	double rms;    
	
	if (useFisheye) 
	{
		Mat _rvecs, _tvecs;
		rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs, _tvecs, flag);    
		rvecs.reserve(_rvecs.rows);
		tvecs.reserve(_tvecs.rows);   
		for(int i = 0; i < int(objectPoints.size()); i++)
		{
			rvecs.push_back(_rvecs.row(i));
			tvecs.push_back(_tvecs.row(i));   
		} 
	}
	else
	{
		rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);
	}
	cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;
	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs, useFisheye);
	return ok;           
}


// Print camera parameters to the output file    
static void saveCameraParams_mono(const string& outputFileName, const int& flag, const float& squareSize, const Size& boardSize, const Size& imageSize, const Mat& cameraMatrix, const Mat& distCoeffs, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints, const double& totalAvgErr, const bool& useFisheye, const bool& writeExtrinsics, const bool& writePoints)

//static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,double totalAvgErr )  
{
	cout << "outputFileName : " << outputFileName << endl;
	FileStorage fs(/*s.*/outputFileName, FileStorage::WRITE );    
	time_t tm; 
	time( &tm );   
	struct tm *t2 = localtime( &tm );     
	char buf[1024];   
	strftime( buf, sizeof(buf), "%c", t2 );      
	fs << "calibration_time" << buf; 
	if( !rvecs.empty() || !reprojErrs.empty() )    
		fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());            
	fs << "image_width" << imageSize.width;  
	fs << "image_height" << imageSize.height;    
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;           
	fs << "square_size" << squareSize;
	/*
	if(flag & CALIB_FIX_ASPECT_RATIO )        
		 fs << "fix_aspect_ratio" << s.aspectRatio;
	*/
	if(flag)          
	{
		std::stringstream flagsStringStream;    
		if (useFisheye)   
		{
			flagsStringStream << "flags:"     
				<< (flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")   
				<< (flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")    
				<< (flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")    
				<< (flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")      
				<< (flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")  
				<< (flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");   
		}
		else
		{
			flagsStringStream << "flags:"      
				<< (flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")        
				<< (flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "") 
				<< (flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")  
				<< (flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")       
				<< (flag & CALIB_FIX_K1 ? " +fix_k1" : "")  
				<< (flag & CALIB_FIX_K2 ? " +fix_k2" : "")  
				<< (flag & CALIB_FIX_K3 ? " +fix_k3" : "")   
				<< (flag & CALIB_FIX_K4 ? " +fix_k4" : "")    
				<< (flag & CALIB_FIX_K5 ? " +fix_k5" : "");   
		}	 
		fs.writeComment(flagsStringStream.str());     
	}
	fs << "flags" << flag;          
	fs << "fisheye_model" << useFisheye;                                                       
	fs << "camera_matrix" << cameraMatrix;  
	fs << "distortion_coefficients" << distCoeffs;  
	fs << "avg_reprojection_error" << totalAvgErr;           
	if (writeExtrinsics && !reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);    
	if(writeExtrinsics && !rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));   
		bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
		bool needReshapeT = tvecs[0].depth() != 1 ? true : false; 
		for( size_t i = 0; i < rvecs.size(); i++ ) 
		{
			Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));   
			Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));  
			if(needReshapeR)  
				rvecs[i].reshape(1, 1).copyTo(r);     
			else 
			{
				 //*.t() is MatExpr (not Mat) so we can use assignment operator     
				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);    
				r = rvecs[i].t();            
			}
			if(needReshapeT)    
				tvecs[i].reshape(1, 1).copyTo(t);       
			else     
			{
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1); 
				t = tvecs[i].t(); 
			}
		}
		fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");      
		fs << "extrinsic_parameters" << bigmat;         
	} 
	if(writePoints && !imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);    
		for( size_t i = 0; i < imagePoints.size(); i++ )  
		{
			Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);       
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);  
		}
		fs << "image_points" << imagePtMat;           
	}               
	return;
}


bool runCalibrationAndSave_mono(const string& fn_yml, const float& square_size, const Size& board_size, const Size& imageSize, const vector<vector<Point3f> > objectPoints, const vector<vector<Point2f> >& imagePoints, const int& flag, const bool& is_fisheye)
{
	bool write_extr = false, write_img_point = false;
	Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;
	vector<Point3f> newObjPoints;

	bool ok = runCalibration_mono(cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, totalAvgErr, imageSize, imagePoints, objectPoints, flag, is_fisheye);//, newObjPoints, grid_width, release_object);

	cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = " << totalAvgErr << endl;

	if (ok)
		saveCameraParams_mono(fn_yml, flag, square_size, board_size, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr, is_fisheye, write_extr, write_img_point);

	return ok;

}

bool runCalibrationAndSave_stereo(const string& dir_img, const vector<vector<Point3f> >& objectPoints,
	const vector<vector<vector<Point2f> > > li_li_image_point, const Size& imageSize, const string& dir_calib,
	const string& postphix, const double& alfa, const bool& showRectified, const bool& useCalibrated, 
	const vector<string>& goodImageList)
{
	int i, j, k, nimages = objectPoints.size(), npoints = 0;
	Mat R, T, E, F, cameraMatrix[2], distCoeffs[2];
	for(i = 0; i < 2; i++) 
	{
		cameraMatrix[i] = initCameraMatrix2D(objectPoints, li_li_image_point[i], imageSize, 0);
	}
	double rms = stereoCalibrate(objectPoints, li_li_image_point[0], li_li_image_point[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    CALIB_FIX_ASPECT_RATIO +
                    CALIB_ZERO_TANGENT_DIST +
                    //CALIB_USE_INTRINSIC_GUESS +
                    //CALIB_SAME_FOCAL_LENGTH +
                    CALIB_RATIONAL_MODEL +
                    CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5 + CALIB_FIX_K6,
                    TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
	cout << "done with RMS error=" << rms << endl;


	bool 
	is_camera_matrix_ok_l = checkRange(cameraMatrix[0]),
	is_camera_matrix_ok_r = checkRange(cameraMatrix[1]),
	is_distotion_ok_l = checkRange(distCoeffs[0]),
	is_distotion_ok_r = checkRange(distCoeffs[1]);

	if(!(is_camera_matrix_ok_l && is_camera_matrix_ok_r && is_distotion_ok_l && is_distotion_ok_r))
	{
		return false;
	}

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	vector<Vec3f> lines[2];
	for( i = 0; i < nimages; i++ )
	{
		//int npt = (int)imagePoints[0][i].size();
		int npt = (int)li_li_image_point[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
        	imgpt[k] = Mat(li_li_image_point[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
        	double errij = fabs(li_li_image_point[0][i][j].x * lines[1][j][0] +
                                li_li_image_point[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
                           fabs(li_li_image_point[1][i][j].x * lines[0][j][0] +
                                li_li_image_point[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " <<  err/npoints << endl;

    // save intrinsic parameters
    std::string path_intrinsic = python_join_equivalent(dir_calib, "intrinsics_" + postphix + ".yml");
    FileStorage fs(path_intrinsic, FileStorage::WRITE);
    //FileStorage fs(python_join_equivalent(dir_calib, "intrinsics.yml"), FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            	"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
        cout << "Just saved the intrinsic parameters at " << path_intrinsic << endl;
    }
    else
    {
        cout << "Error: can not save the intrinsic parameters\n";
        exit(0);
    }

    std::cout << endl << "cameraMatrix[0] : " << endl << cameraMatrix[0] << endl; 
    std::cout << endl << "distCoeffs[0] : " << endl << distCoeffs[0] << endl; 
    std::cout << endl << "cameraMatrix[1] : " << endl << cameraMatrix[1] << endl; 
    std::cout << endl << "distCoeffs[1] : " << endl << distCoeffs[1] << endl; 
    std::cout << endl << "R : " << endl << R << endl; 
    std::cout << endl << "T : " << endl << T << endl << endl; 
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, alfa, imageSize, &validRoi[0], &validRoi[1]);
	
    std::string path_extrinsic = python_join_equivalent(dir_calib, "extrinsics_" + postphix + ".yml");
    fs.open(path_extrinsic, FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
        cout << "Just saved the extrinsic parameters at " << path_extrinsic << endl;
    }
    else
    {
        cout << "Error: can not save the extrinsic parameters\n";
        exit(0);
    }

    if( !showRectified )
 	// COMPUTE AND DISPLAY RECTIFICATION
        return true;

   // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    cout << "(P2.at<double>(1, 3)) : " << P2.at<double>(1, 3) << endl;
    cout << "(P2.at<double>(0, 3)) : " << P2.at<double>(0, 3) << endl;
    cout << "isVerticalStereo : " << isVerticalStereo << endl;
    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
	if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(li_li_image_point[k][i].begin(), li_li_image_point[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;//, canvas_ori;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w * 2, CV_8UC3);
        //canvas_ori.create(imageSize.height, imageSize.width * 2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
        //canvas_ori.create(imageSize.height * 2, imageSize.width, CV_8UC3);
    }	
    cout << "useCalibrated : " << useCalibrated << endl;
    stringstream ss;
    for( i = 0; i < nimages; i++ )
    {
        cv::Mat canvas_ori(imageSize.height, imageSize.width * 2, CV_8UC1);
		for( k = 0; k < 2; k++ )
        {
            //Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            string fn = goodImageList[i * 2 + k];
            cout << "fn : " << fn << endl;
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            //imshow("img", img); imshow("rimg", rimg); waitKey();
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            img.copyTo(canvas_ori(cv::Rect(0 == k ? 0 : img.cols, 0, img.cols, img.rows)));
            //imshow("original", canvas_ori); waitKey();
            
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }
        //canvas_ori.adjustROI(0, 0, imageSize.width, 0);
        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        char buff[100];

        snprintf(buff, sizeof(buff), "rectified_%02d.png", i + 1);
        std::string buffAsStdStr = buff;
        buffAsStdStr = python_join_equivalent(dir_img, buffAsStdStr);
        imwrite(buffAsStdStr, canvas);
        cout << buffAsStdStr << " is just saved" << endl;
        imshow("rectified", canvas);
        imshow("original", canvas_ori);
		//char c = (char)waitKey(7000);
        char c = (char)waitKey(11000);
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
	}
	return true;
}
 

static void
StereoCalib(const vector<string>& imagelist, const string& dir_img, const string& dir_calib, const string& postphix, Size& boardSize, float squareSize, vector<int> li_one_based_indices_to_skip, const double& alfa, bool is_mono, bool is_fisheye, bool displayCorners = false, bool useCalibrated=true, bool showRectified=true)
{
    if(!is_mono && imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;
	//int radius = 2; 
	int radius = 20; 
    // ARRAY AND VECTOR STORAGE:

    //vector<vector<Point2f> > li_li_image_point[2];
    vector<vector<vector<Point2f> > > li_li_image_point;
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages, flag_mono = 0, n_list = is_mono ? 1 : 2; 
	nimages = (int)imagelist.size() / n_list;
	li_li_image_point.resize(n_list);
  	if(is_mono)
	{
		flag_mono |= CV_CALIB_FIX_K4;	flag_mono |= CV_CALIB_FIX_K5;
	}
    vector<string> goodImageList;
    //cout << "dir_img : " << dir_img << endl;
    cout << "nimages : " << nimages << endl;
    for( i = j = 0; i < nimages; i++ )
    {
        bool is_in_skipping_list = std::find(li_one_based_indices_to_skip.begin(), li_one_based_indices_to_skip.end(), i + 1) != li_one_based_indices_to_skip.end();
        if (is_in_skipping_list) continue; 
        for( k = 0; k < n_list; k++ )
        {
            const string& filename = imagelist[i * n_list + k];
            std::string path_img = python_join_equivalent(dir_img, filename);
            cout << endl << "path_img : " << path_img << endl;
            Mat img = imread(path_img, 0);
            //Mat img = imread(filename, 0);
            if(img.empty())
            {
                cout << "Can not read image file : " << path_img << endl;
                break;
            }
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f> li_corner;
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale, INTER_LINEAR/*_EXACT*/);
                found = findChessboardCorners(timg, boardSize, li_corner,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if(found)
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(li_corner);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            vector<Point2f> li_corner_subpxl(li_corner);
            //  찾았으면
            if(found)
            {
                //  subpixel로 찾는다
                //TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001)
                TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001);

                cornerSubPix(img, li_corner_subpxl, Size(6, 6), Size(-1,-1), criteria);
                //  subpixel 결과를 리스트에 넣는다.
                li_li_image_point[k].push_back(li_corner_subpxl);
                //std::cout << "Grid corners are found in this image !!!" << std::endl;
            }
            else
            {
                std::cout << "Grid corners are NOT found in this image !!!" << std::endl;
            }
            //  코너점들을 그린다.
            if (displayCorners) draw_chessboard_grids(img, found, is_in_skipping_list, 0 == k, i + 1, nimages, radius, boardSize, li_corner, li_corner_subpxl);
        }
        if(n_list == k)
        //if(!is_in_skipping_list &&  2 == k)
        {
			for(int iL = 0; iL < n_list; iL++)
			{
            	goodImageList.push_back(python_join_equivalent(dir_img, imagelist[i * n_list + iL]));
			}
            j++;
        }
        else
        {
            std::cout << "The pair of images is NOT included in googImageList !!" << std::endl;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 7 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }
    objectPoints.resize(nimages);
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
		{
            for( k = 0; k < boardSize.width; k++ )
			{
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
			}
		}
    }

    cout << "Running stereo calibration ...\n";
/*
    //Mat cameraMatrix[2], distCoeffs[2];
    vector<Mat> cameraMatrix, distCoeffs;
	cameraMatrix.resize(n_list);	distCoeffs.resize(n_list);
	for(int iL = 0; iL < n_list; iL++)
	{
	
    	cameraMatrix[iL] = initCameraMatrix2D(objectPoints, li_li_image_point[iL], imageSize, 0);
	}
    Mat R, T, E, F;
*/	
	if (is_mono)
	{
		string fn_yml_mono = python_join_equivalent(dir_calib, "cam_param_" + postphix + ".yml"); 
		std::cout << "fn_yml_mono : " << fn_yml_mono << std::endl;
		runCalibrationAndSave_mono(fn_yml_mono, squareSize, boardSize, imageSize, objectPoints, li_li_image_point[0], flag_mono, is_fisheye);
	}
	else
	{
		runCalibrationAndSave_stereo(dir_img, objectPoints, li_li_image_point, imageSize, dir_calib, postphix, alfa, showRectified, useCalibrated, goodImageList);
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
    vector<int> li_one_based_indices_to_skip;
    Size boardSize;
    string postphix, one_based_indices_to_skip, dir_img, dir_calib, imagelistfn;
    bool showRectified;
    //cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{s|1.0|}{nr||}{help||}{@input|../data/stereo_calib.xml|}");
    cv::CommandLineParser parser(argc, argv, "{fish||If the camera lens is fisheye}     {mono||If the camera is mono}     {postfix||string that shows the calibration settings}     {dir_calib||the directory of yml files of the intrinsic and extrinsic stereo camera parameters}     {alfa|0|alpha value for stereoRectify function}     {e||list of image indices which should be not processes (one based). For example, -e=1,8,14 for skipping the first, 7th, and last stereo images of a 14-image-set}     {w||the number of cols in the chessboard. Actually #cols + 1}     {h||the number of rows in the chessboard. Actually #rows + 1}     {s||the physical side length of a cell in the chessboard}{nr||short for no rectification}{help||}     {dir_img||the path to the left and right image pairs}{input||path to xml file of input image list}");

    if (parser.has("help"))
        return print_help();
    bool is_mono = parser.has("mono");
    bool is_fisheye = parser.has("fish");
    showRectified = !parser.has("nr");
    imagelistfn = parser.get<string>("input");
    cout << "imagelistfn : " << imagelistfn << endl;
    one_based_indices_to_skip = parser.get<string>("e");
    if(!one_based_indices_to_skip.empty())
    {
        li_one_based_indices_to_skip = get_one_based_skipping_indices(one_based_indices_to_skip);
    }
    dir_img = parser.get<string>("dir_img");
    cout << "dir_img : " << dir_img << endl;
    dir_calib = parser.get<string>("dir_calib");
    cout << "dir_calib : " << dir_calib << endl;
    postphix = parser.get<string>("postfix");
    cout << "postphix : " << postphix << endl;
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    float squareSize = parser.get<float>("s");
    double alfa = parser.get<float>("alfa");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    
    if(!mkdir_if_not_exist(dir_calib.c_str()))
    {
        exit(0);
    };
    vector<string> imagelist;
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
        return print_help();
    }

    //StereoCalib(imagelist, dir_img, boardSize, squareSize, false, true, showRectified);
    StereoCalib(imagelist, dir_img, dir_calib, postphix, boardSize, squareSize, li_one_based_indices_to_skip, alfa, is_mono, is_fisheye, true, true, showRectified);
    return 0;
}
