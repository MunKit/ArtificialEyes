#pragma once
#include <iostream>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

float respondToUser(int &nearest_distance, Mat &scene, char section);

float lowspeed = 0.3;
float highspeed = 0.9;
Mat generateDisparityMapU(Mat &disp_img, int max_disp)
{
	Mat img = disp_img.clone();

	resize(img, img, Size(), 0.25, 0.25, CV_INTER_AREA);

	int IMAGE_HEIGHT = img.rows;
	int IMAGE_WIDTH = img.cols;
	int MAX_DISP = max_disp;

	float value_ranges[] = { (float)0, (float)MAX_DISP };
	const float* hist_ranges[] = { value_ranges };
	int channels[] = { 0 };
	int histSize[] = { MAX_DISP };

	Mat u_disp = Mat::zeros(MAX_DISP, IMAGE_WIDTH, CV_32F);
	Mat tmp_img_mat, tmp_hist_mat;

	img = img.t();  // transpose the image for fast processing
	for (int i = 0; i < IMAGE_WIDTH; i++)
	{
		//tmp_img_mat = img.col(i);  // for no transpose image
		tmp_img_mat = img.row(i);  // for transpose image
		u_disp.col(i).copyTo(tmp_hist_mat);

		calcHist(&tmp_img_mat, 1, channels, cv::Mat(), tmp_hist_mat, 1, histSize, hist_ranges, true, false);

		u_disp.col(i) = tmp_hist_mat / (float)IMAGE_WIDTH;
	}
	img = img.t();  // transpose the image to get back original image

	u_disp.convertTo(u_disp, CV_8U, 255);

	return u_disp;
}

void eliminateGroundPlane(Mat &process_disp_img, Mat &u_disp, int alarm_range, int min_obtacle_pix)
{
	resize(process_disp_img, process_disp_img, Size(), 0.25, 0.25, CV_INTER_AREA);

	int ALARM_RANGE = alarm_range;

	// Step 1: Set obstacle alarm range.
	uchar out_of_range_value = (uchar)0;
	for (int j = 0; j < ALARM_RANGE; j++)
		for (int i = 0; i < u_disp.cols; i++)
			u_disp.at<uchar>(j, i) = out_of_range_value;

	// Step 2: Remove ground plane pixel and non-obstacle pixel from U-disparity map.
	threshold(u_disp, u_disp, min_obtacle_pix, 255, THRESH_TOZERO);

	// Step 3: Remove ground plane pixel and non-obstacle pixel from disparity map.
	vector<int> preserved_disp_val;
	for (int i = 0; i < u_disp.cols; i++)
	{
		for (int j = 0; j < u_disp.rows; j++)
		{
			Scalar u_pix_pos = u_disp.at<uchar>(j, i);
			uchar u_pix_val = u_pix_pos.val[0];
			int u_value = u_pix_val;
			if (u_value != 0)
				preserved_disp_val.push_back(j);
		}

		sort(preserved_disp_val.begin(), preserved_disp_val.end());

		for (int j = 0; j < process_disp_img.rows; j++)
		{
			Scalar disp_pix_pos = process_disp_img.at<uchar>(j, i);
			uchar disp_pix_val = disp_pix_pos.val[0];
			int disp_value = disp_pix_val;
			bool search_state = binary_search(preserved_disp_val.begin(), preserved_disp_val.end(), disp_value);
			if (!search_state)
			{
				uchar new_disp_pix_value = (uchar)0;
				process_disp_img.at<uchar>(j, i) = new_disp_pix_value;
			}
		}
		preserved_disp_val.clear();
	}
}

void postProcessingDisparityMap(Mat &process_disp_img)
{
	threshold(process_disp_img, process_disp_img, 0, 255, THRESH_BINARY);

	Mat element1 = getStructuringElement(MORPH_RECT, Size(5, 5), Point(0, 0));
	erode(process_disp_img, process_disp_img, element1);
	Mat element2 = getStructuringElement(MORPH_RECT, Size(7, 7), Point(0, 0));
	dilate(process_disp_img, process_disp_img, element2);
	Mat element3 = getStructuringElement(MORPH_RECT, Size(7, 7), Point(0, 0));
	morphologyEx(process_disp_img, process_disp_img, MORPH_CLOSE, element3);

	resize(process_disp_img, process_disp_img, Size(), 4, 4, CV_INTER_AREA);
}

vector<float> disparityToDistance(Mat &process_disp_img, Mat &disp_img, Mat &scene, Mat &Q)
{
	Mat temp_disp_img = disp_img.clone();

	Mat contours_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Step 1: Find the obstacles contours.
	contours_output = process_disp_img.clone();
	findContours(contours_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Step 2: Calculate moments.
	vector<Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	// Step 3: Calculate center of mass.
	vector<Point2f> center_of_mass(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		center_of_mass[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	//Step 4: Divide and draw the environment scene into three section.
	rectangle(scene, Point(0, 0), Point(209, 479), Scalar(255, 0, 0), 2, 8, 0);
	rectangle(scene, Point(210, 0), Point(429, 479), Scalar(255, 0, 0), 2, 8, 0);
	rectangle(scene, Point(430, 0), Point(639, 479), Scalar(255, 0, 0), 2, 8, 0);
	rectangle(disp_img, Point(0, 0), Point(209, 479), Scalar(255, 0, 0), 2, 8, 0);
	rectangle(disp_img, Point(210, 0), Point(429, 479), Scalar(255, 0, 0), 2, 8, 0);
	rectangle(disp_img, Point(430, 0), Point(639, 479), Scalar(255, 0, 0), 2, 8, 0);

	// Step 5: Calculate obstacle's distance.
	vector<float> distance(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		Scalar intensity = temp_disp_img.at<uchar>(center_of_mass[i].y, center_of_mass[i].x);
		uchar grayscale = intensity.val[0];

		float fMaxDistance = static_cast<float>((1. / Q.at<double>(3, 2)) * 1000 * Q.at<double>(2, 3));
		int outputDisparityValue = grayscale;

		float distance_constant = 0.0;
		if (outputDisparityValue > 147)
			distance_constant = 4.2660;
		else if (outputDisparityValue >130 && outputDisparityValue <= 147)
			distance_constant = 4.7318;
		else if (outputDisparityValue >120 && outputDisparityValue <= 130)
			distance_constant = 4.6525;
		else if (outputDisparityValue >110 && outputDisparityValue <= 120)
			distance_constant = 4.7241;
		else if (outputDisparityValue >101 && outputDisparityValue <= 110)
			distance_constant = 4.7241;
		else if (outputDisparityValue >94 && outputDisparityValue <= 101)
			distance_constant = 4.6990;
		else if (outputDisparityValue >85 && outputDisparityValue <= 94)
			distance_constant = 4.7097;
		else if (outputDisparityValue >80 && outputDisparityValue <= 85)
			distance_constant = 4.5630;
		else if (outputDisparityValue >74 && outputDisparityValue <= 80)
			distance_constant = 4.5809;
		else if (outputDisparityValue >68 && outputDisparityValue <= 74)
			distance_constant = 4.5024;
		else if (outputDisparityValue >66 && outputDisparityValue <= 68)
			distance_constant = 4.3805;
		else if (outputDisparityValue >60 && outputDisparityValue <= 66)
			distance_constant = 4.7241;
		else if (outputDisparityValue >57 && outputDisparityValue <= 60)
			distance_constant = 4.5093;
		else if (outputDisparityValue >54 && outputDisparityValue <= 57)
			distance_constant = 4.4879;
		else if (outputDisparityValue >52 && outputDisparityValue <= 54)
			distance_constant = 4.4449;
		else if (outputDisparityValue >48 && outputDisparityValue <= 52)
			distance_constant = 4.4664;
		else if (outputDisparityValue =48)
			distance_constant = 4.2946;
		
		
		float fDisparity = outputDisparityValue / distance_constant;  // (float)StereoMatcher::DISP_SCALE; //4.7
		float fDistance = (fMaxDistance / (fDisparity * 10.0));  // "10.0" is used to convert mm to cm
		distance[i] = fDistance;
	}

	// Step 6: Draw bounding rectangle surround obstacles and label the distance that is less than 400cm and draw center of mass.
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}

	for (int i = 0; i< contours.size(); i++)
	{
		if (distance[i] <= 500)
		{
			ostringstream ss;  // convert float to string
			ss << distance[i];
			string s(ss.str());

			putText(scene, s + "cm", center_of_mass[i], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2.0, 8, false);
			rectangle(scene, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 255), 2, 8, 0);
			rectangle(disp_img, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 255, 255), 2, 8, 0);
			circle(scene, center_of_mass[i], 4, Scalar(255, 0, 255), -1, 8, 0);
			circle(disp_img, center_of_mass[i], 4, Scalar(0, 0, 0), -1, 8, 0);
		}
	}

	// Step 7: Analyse nearest obstacle in each scene's section and respond to blind user.
	int left_section_x1 = 0;
	int left_section_x2 = 209;
	int middle_section_x1 = 210;
	int middle_section_x2 = 429;
	int right_section_x1 = 430;
	int right_section_x2 = 639;

	int left_section_nearest_distance = 501;
	int middle_section_nearest_distance = 501;
	int right_section_nearest_distance = 501;

	for (int i = 0; i < contours.size(); i++)
	{
		int x_left = boundRect[i].x;
		int x_right = boundRect[i].x + boundRect[i].width;

		if ((x_left >= left_section_x1 && x_left <= left_section_x2)
			|| (x_right >= left_section_x1 && x_right <= left_section_x2))
		{
			if (distance[i] < left_section_nearest_distance)
				left_section_nearest_distance = distance[i];
		}

		if ((x_left >= middle_section_x1 && x_left <= middle_section_x2)
			|| (x_right >= middle_section_x1 && x_right <= middle_section_x2))
		{
			if (distance[i] < middle_section_nearest_distance)
				middle_section_nearest_distance = distance[i];
		}

		if ((x_left >= right_section_x1 && x_left <= right_section_x2)
			|| (x_right >= right_section_x1 && x_right <= right_section_x2))
		{
			if (distance[i] < right_section_nearest_distance)
				right_section_nearest_distance = distance[i];
		}
	}
	
	
	float message_L = respondToUser(left_section_nearest_distance, scene, 'L');
	float message_M = respondToUser(middle_section_nearest_distance, scene, 'M');
	float message_R = respondToUser(right_section_nearest_distance, scene, 'R');
	
	vector<float> message;
	message.push_back(message_L);
	message.push_back(message_M);
	message.push_back(message_R);
	return message;
}

float respondToUser(int &nearest_distance, Mat &scene, char section)
{
	Point text_pos;
	if (section == 'L')
		text_pos = Point(40, 40);
	else if (section == 'M')
		text_pos = Point(250, 40);
	else if (section == 'R')
		text_pos = Point(470, 40);
	
	float message = 0.0;
	//cout<<nearest_distance<<endl;
	if (nearest_distance <= 500 && nearest_distance > 250)
	{
		putText(scene, "Slow Vibration", text_pos, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2.0, 8, false);
		message = lowspeed;
	}
	else if (nearest_distance <= 250 && nearest_distance > 0)
	{
		putText(scene, "Fast Vibration", text_pos, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2.0, 8, false);
		message = highspeed;
	}
	return message;
}
