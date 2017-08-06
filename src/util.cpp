#pragma once
#include "util.h"

inline void world_to_image_vec(std::vector<float>& hh, std::vector<float>& pp) {
	// don't touch
	float height_ = 720.f;
	float IMG_V_ = 360.f;
	float a = IMG_V_;			// y coordinate of the Vanishing Point
	float ten_meter = IMG_V_ - 321.f;	// y coordinate of the imaginary 10-meter ahead
	float b = a * 10.f / ten_meter - 10.f;

	pp.resize(hh.size());
	for (int ii = 0; ii < hh.size(); ++ii) {
		pp[ii] = IMG_V_ - ((a*hh[ii]) / (float)(b + hh[ii]));
	}
}

inline float world_to_image(float hh) {
	// don't touch
	float height_ = 720.f;
	float IMG_V_ = 360.f;
	float a = IMG_V_;			// y coordinate of the Vanishing Point
	float ten_meter = IMG_V_ - 321.f;	// y coordinate of the imaginary 10-meter ahead
	float b = a * 10.f / ten_meter - 10.f;
	float temp_val = IMG_V_ - ((a*hh) / (float)(b + hh));
	return temp_val;
}
void polynomial_initialize(POLYNOMIAL& poly, int n_poly){

	poly.coeff.resize(n_poly);
	poly.avg_p = cv::Point2f(0, 0);
	poly.scale = 1.f;

	poly.img_to_world;
	//poly.world_to_image;

	// don't touch
	float height_ = 720.f;
	float IMG_V_ = 360.f;
	float a = IMG_V_;			// y coordinate of the Vanishing Point
	float ten_meter = IMG_V_ - 321.f;	// y coordinate of the imaginary 10-meter ahead
	float b = a * 10.f / ten_meter - 10.f;
	float upper_y = b*(IMG_V_ - 240.f) / (a - (IMG_V_ - 240.f));
	float lower_y = b*(IMG_V_ - height_) / (a - (IMG_V_ - height_));

	// height to world
	poly.img_to_world.resize(height_+1);
	for (float y = 1; y <= height_; ++y) {
		float temp_val = (IMG_V_ - y) * b / (a + y - IMG_V_);
		poly.img_to_world[y] = temp_val;
	}

}

float calculate_fitting_error(const std::vector<cv::Point2f>& points, const POLYNOMIAL& poly) {

	float err_fit = 0.f;
	int n_samples = points.size();

	if (poly.coeff.size() == 3) {

		for (int ii = 0; ii < n_samples; ++ii) {

			float yy_norm = (poly.img_to_world[(int)points[ii].y] - poly.avg_p.y) / poly.scale;
			float xx_norm = 0.f;
			for (int cc = (int)poly.coeff.size() - 1; cc >= 0; --cc) {
				xx_norm = xx_norm * yy_norm + poly.coeff[cc];
			}
			err_fit += fabsf(xx_norm * poly.scale + poly.avg_p.x - points[ii].x);
		}
	}
	else {
		for (int ii = 0; ii < n_samples; ++ii) {

			float yy_norm = (points[ii].y - poly.avg_p.y) / poly.scale;
			float xx_norm = 0.f;
			for (int cc = (int)poly.coeff.size() - 1; cc >= 0; --cc) {
				xx_norm = xx_norm * yy_norm + poly.coeff[cc];
			}
			err_fit += fabsf(xx_norm * poly.scale + poly.avg_p.x - points[ii].x);
		}

	}

	return err_fit / ((float)points.size());
}

void fittingLinesWithPoly(std::vector<float> pts_y, std::vector<float>& pts_x, POLYNOMIAL poly){

	int n_samples = pts_y.size();
	pts_x.resize(n_samples);

	if (poly.coeff.size() == 3) {
		for (int ii = 0; ii < n_samples; ++ii) {

			float yy_norm = (poly.img_to_world[(int)pts_y[ii]] - poly.avg_p.y) / poly.scale;

			float xx_norm = 0.f;
			for (int cc = (int)poly.coeff.size() - 1; cc >= 0; --cc) {
				xx_norm = xx_norm * yy_norm + poly.coeff[cc];
			}
			pts_x[ii] = xx_norm * poly.scale + poly.avg_p.x;
		}
	}
	else {
		for (int ii = 0; ii < n_samples; ++ii) {

			float yy_norm = (pts_y[ii] - poly.avg_p.y) / poly.scale;

			float xx_norm = 0.f;
			for (int cc = (int)poly.coeff.size() - 1; cc >= 0; --cc) {
				xx_norm = xx_norm * yy_norm + poly.coeff[cc];
			}
			pts_x[ii] = xx_norm * poly.scale + poly.avg_p.x;
		}
	}
}

float fittingPointWithPoly(float pts_y, POLYNOMIAL poly){
	
	float yy_norm;
	if (poly.coeff.size() == 3) {
		yy_norm = (poly.img_to_world[(int)pts_y] - poly.avg_p.y) / poly.scale;
	}
	else {
		yy_norm = (pts_y - poly.avg_p.y) / poly.scale;
	}

	float xx_norm = 0.f;
	for (int cc = (int)poly.coeff.size() - 1; cc >= 0; --cc) {
		xx_norm = xx_norm * yy_norm + poly.coeff[cc];
	}
	return (xx_norm * poly.scale + poly.avg_p.x);
}



float getPoly2_norm(std::vector<cv::Point2f> points, int n, POLYNOMIAL& poly) {

	// image to world
	//std::vector<cv::Point2f> points(points_img.size());
	//for (int ii = 0; ii < points_img.size(); ++ii) {
	//	points[ii] = cv::Point2f(points_img[ii].x, poly.img_to_world[points_img[ii].y]);
	//}

	float avg_x = 0.f;
	float avg_y = 0.f;
	for (int ii = 0; ii < n; ++ii) {
		avg_x += points[ii].x;
		avg_y += points[ii].y;
	}
	avg_x = avg_x / (float)n;
	avg_y = avg_y / (float)n;

	float max_mag = 0.f;
	for (int ii = 0; ii < n; ++ii) {
		float dist = fabsf(points[ii].x - avg_x) + fabsf(points[ii].y - avg_y);
		max_mag = std::max(dist, max_mag);
	}
	poly.avg_p = cv::Point2f(avg_x, avg_y);
	poly.scale = max_mag;

	float temp;
	double err;
	cv::Mat a = cv::Mat(2, 2, CV_32FC1);
	cv::Mat b = cv::Mat(2, 1, CV_32FC1);
	cv::Mat c = cv::Mat(2, 2, CV_32FC1);
	cv::Mat d = cv::Mat(2, 1, CV_32FC1);
	cv::Mat e = cv::Mat(2, 1, CV_32FC1);

	for (int ii = 0; ii < n; ii++) {
		points[ii].x = (points[ii].x - avg_x) / poly.scale;
		points[ii].y = (points[ii].y - avg_y) / poly.scale;
	}
	// configuring matrix 'a'
	a.at<float>(0, 0) = (float)n;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].y;
	}
	a.at<float>(0, 1) = (float)temp;
	a.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].y * points[ii].y;
	}
	a.at<float>(1, 1) = (float)temp;
	//temp = 0;
	//for (int ii = 0; ii < n; ii++) {
	//	temp += points[ii].y * points[ii].y * points[ii].y;
	//}

	// configuring matrix 'b'
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].x;
	}
	b.at<float>(0, 0) = (float)temp;
	temp = 0;
	for (int ii = 0; ii < n; ii++) {
		temp += points[ii].x * points[ii].y;
	}
	b.at<float>(1, 0) = (float)temp;

	// matrix operation
	c = a.inv();
	d = c*b;
	poly.coeff[0] = d.at<float>(0, 0);
	poly.coeff[1] = d.at<float>(1, 0);

	//printf("%f %f %f\n", coeff[0], coeff[1], coeff[2]); 

	e = a*d;
	err = abs(e.at<float>(0, 0) - b.at<float>(0, 0)) + abs(e.at<float>(1, 0) - b.at<float>(1, 0));

	return err;
}

float getPoly3_norm(std::vector<cv::Point2f> points_img, int n, POLYNOMIAL& poly){
	
	// image to world
	std::vector<cv::Point2f> points(points_img.size());
	for (int ii = 0; ii < points_img.size(); ++ii) {
		points[ii] = cv::Point2f(points_img[ii].x, poly.img_to_world[points_img[ii].y]);
	}

	float avg_x = 0.f;
	float avg_y = 0.f;
	for (int ii = 0; ii < n; ++ii) {
		avg_x += points[ii].x;
		avg_y += points[ii].y;
	}
	avg_x = avg_x / (float)n;
	avg_y = avg_y / (float)n;
	
	float max_mag = 0.f;
	for (int ii = 0; ii < n; ++ii) {
		float dist = fabsf(points[ii].x - avg_x) + fabsf(points[ii].y - avg_y);
		max_mag = std::max(dist, max_mag);
	}
	poly.avg_p = cv::Point2f(avg_x, avg_y);
	poly.scale = max_mag;
	
	float temp;
	float err = 0;
	cv::Mat a = cv::Mat(3, 3, CV_32FC1);
	cv::Mat b = cv::Mat(3, 1, CV_32FC1);
	cv::Mat c = cv::Mat(3, 3, CV_32FC1);
	cv::Mat d = cv::Mat(3, 1, CV_32FC1);
	cv::Mat e = cv::Mat(3, 1, CV_32FC1);

	for (int ii = 0; ii < n; ii++) {
		points[ii].x = (points[ii].x - avg_x) / poly.scale;
		points[ii].y = (points[ii].y - avg_y) / poly.scale;
	}
	// configuring matrix 'a'
	a.at<float>(0, 0) = (float)n;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y;
	}
	a.at<float>(0, 1) = (float)temp;
	a.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y;
	}
	a.at<float>(0, 2) = (float)temp;
	a.at<float>(1, 1) = (float)temp;
	a.at<float>(2, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(1, 2) = (float)temp;
	a.at<float>(2, 1) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].y * points[i].y;
	}
	a.at<float>(2, 2) = (float)temp;
	
	// configuring matrix 'b'
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x;
	}
	b.at<float>(0, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].x * points[i].y;
	}
	b.at<float>(1, 0) = (float)temp;
	temp = 0;
	for (int i = 0; i < n; i++) {
		temp += points[i].y * points[i].y * points[i].x;
	}
	b.at<float>(2, 0) = (float)temp;
	
	// matrix operation	
	d = a.inv()*b;
	
	poly.coeff[0] = d.at<float>(0, 0);
	poly.coeff[1] = d.at<float>(1, 0);
	poly.coeff[2] = d.at<float>(2, 0);
	
	e = a*d;
	err = abs(e.at<float>(0, 0) - b.at<float>(0, 0)) + abs(e.at<float>(1, 0) - b.at<float>(1, 0)) + abs(e.at<float>(2, 0) - b.at<float>(2, 0));

	return 0.f;
}

void getTransformedPoints(const std::vector<cv::Point2f>& srcPoints, const cv::Mat& state, std::vector<cv::Point2f>& dstPoints) {

	// faster version -- 5.5(s) -> 1.7(s)
	int size = (int)srcPoints.size();
	if (size == 0) {
		std::cout << " ERROR imageOperator.cpp : srcPoints size is 0" << std::endl;
	}
	dstPoints.resize(size);

	for (int pp = 0; pp < size; ++pp) {

		float resX = state.ptr<float>(0)[0] * srcPoints[pp].x + state.ptr<float>(0)[1] * srcPoints[pp].y + state.ptr<float>(0)[2];
		float resY = state.ptr<float>(1)[0] * srcPoints[pp].x + state.ptr<float>(1)[1] * srcPoints[pp].y + state.ptr<float>(1)[2];
		float normconst = state.ptr<float>(2)[0] * srcPoints[pp].x + state.ptr<float>(2)[1] * srcPoints[pp].y + state.ptr<float>(2)[2];
		dstPoints[pp] = cv::Point2f(resX / normconst, resY / normconst);
	}
}