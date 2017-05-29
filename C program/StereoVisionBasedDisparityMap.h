#pragma once
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

void readStereoCalibrationFile(string calib_filename, Mat &K_left, Mat &K_right, Mat &D_left, Mat &D_right, Mat &R_left, Mat &R_right, Mat &P_left, Mat &P_right, Mat &Q)
{
	FileStorage fs1(calib_filename, FileStorage::READ);
	fs1["K_left"] >> K_left;
	fs1["K_right"] >> K_right;
	fs1["D_left"] >> D_left;
	fs1["D_right"] >> D_right;
	fs1["R_left"] >> R_left;
	fs1["R_right"] >> R_right;
	fs1["P_left"] >> P_left;
	fs1["P_right"] >> P_right;
	fs1["Q"] >> Q;
}


void stereoRectifytImage(Mat &left_camera_frame, Mat &right_camera_frame, Mat &K_left, Mat &K_right, Mat &D_left, Mat &D_right, Mat &R_left, Mat &R_right, Mat &P_left, Mat &P_right)
{
	Mat lmapx, lmapy, rmapx, rmapy;
	initUndistortRectifyMap(K_left, D_left, R_left, P_left, left_camera_frame.size(), CV_32F, lmapx, lmapy);
	initUndistortRectifyMap(K_right, D_right, R_right, P_right, right_camera_frame.size(), CV_32F, rmapx, rmapy);
	remap(left_camera_frame, left_camera_frame, lmapx, lmapy, INTER_LINEAR);
	remap(right_camera_frame, right_camera_frame, rmapx, rmapy, INTER_LINEAR);
}


void generateDepthMapSGBM(Mat &left_camera_frame, Mat &right_camera_frame, Mat &disparity_map, double vis_mult, double lambda, double sigma, int wsize, int max_disp)
{
	Mat left_for_matcher, right_for_matcher;
	Mat left_disp, right_disp;
	Mat filtered_disp;
	Ptr<DisparityWLSFilter> wls_filter;
	Mat filtered_disp_vis;

	// Step 1: Downscale images to speed up process.
	//max_disp /= 2;
	if (max_disp % 16 != 0)
		max_disp += 16 - (max_disp % 16);
	resize(left_camera_frame, left_for_matcher, Size(), 0.5, 0.5);
	resize(right_camera_frame, right_for_matcher, Size(), 0.5, 0.5);

	// Step 2: Compute disparity map.
	Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, max_disp, wsize);
	left_matcher->setP1(24 * wsize*wsize);
	left_matcher->setP2(96 * wsize*wsize);
	left_matcher->setPreFilterCap(63);
	left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
	wls_filter = createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

	left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
	right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);

	// Step 3: Apply filter
	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);
	wls_filter->filter(left_disp, left_camera_frame, filtered_disp, right_disp);

	getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
	disparity_map = filtered_disp_vis.clone();
}