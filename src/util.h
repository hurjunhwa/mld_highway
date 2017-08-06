#pragma once 
#include <opencv2\opencv.hpp>
#include <iostream>
#include <algorithm>
#include <vector>

struct LANE_MARKING {
	cv::Point2f str_p;
	cv::Point2f cnt_p;
	cv::Point2f end_p;
	cv::Point2f inn_p;
	int size;
};

struct POLYNOMIAL{
	std::vector<float> coeff;
	cv::Point2f avg_p;
	float scale;
	std::vector<float> img_to_world;
};

void polynomial_initialize(POLYNOMIAL& poly, int n_poly);
float calculate_fitting_error(const std::vector<cv::Point2f>& points, const POLYNOMIAL& poly);
void fittingLinesWithPoly(std::vector<float> pts_y, std::vector<float>& pts_x, POLYNOMIAL poly);
float fittingPointWithPoly(float pts_y, POLYNOMIAL poly);
float getPoly2_norm(std::vector<cv::Point2f> points, int n, POLYNOMIAL& poly);
float getPoly3_norm(std::vector<cv::Point2f> points, int n, POLYNOMIAL& poly);
void getTransformedPoints(const std::vector<cv::Point2f>& srcPoints, const cv::Mat& state, std::vector<cv::Point2f>& dstPoints);