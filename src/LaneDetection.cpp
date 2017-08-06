#pragma once
#include "LaneDetection.h"
#include <time.h>
#include <math.h>
#include <json/writer.h>

struct greater
{
	template<class T>
	bool operator()(T const &a, T const &b) const { return a > b; }
};

// Lane segment definition 
#define MAX_LANE_MARKING 2000
#define MAX_LW_N 67		// Max lane width, nearest
#define MAX_LW_F 15		// Max lane width, farest
#define MIN_LW_N 23 		// Min lane width, nearest
#define MIN_LW_F 3			// Min lane width, farest
#define MAX_LW_D 12		// Max lane width, delta
#define SCAN_INTERVAL1 1	//lower
#define SCAN_INTERVAL2 1	//upper

// Lane Marking Grouping
#define MAX_LANE_SEED 200
#define SEED_MARKING_DIST_THRES 90
#define VALID_SEED_MARKING_NUMBER_THRES 5
#define LOW_LEVEL_ASS_THRES 1.96

float length_ftn(cv::Point2f str_p, cv::Point2f end_p) {

	return sqrtf((str_p.x - end_p.x)*(str_p.x - end_p.x) + (str_p.y - end_p.y)*(str_p.y - end_p.y));

}

void LaneDetection::initialize_variable(cv::Mat& img_src, bool mode_color, float ratio_roi) {

	// Image variable 
	width_ = img_src.cols;
	height_ = img_src.rows;
	height_roi_ = (int)(img_src.rows * ratio_roi);
	mode_color_ = mode_color;

	// Inputs for calibration - ROI definition: should be revised
	{
		v_idx_.resize(0);

		IMG_U_ = (int)(width_ / 2.f);
		IMG_V_ = (int)(height_ / 2.f);

		float a = IMG_V_ - 224.f;			// y coordinate of the Vanishing Point
		float ten_meter = IMG_V_ - 321.f;	// y coordinate of the imaginary one-meter ahead
		float b = a * 10.f / ten_meter - 10.f;

		float upper_y = b*(IMG_V_ - height_roi_) / (a - (IMG_V_ - height_roi_));
		float lower_y = b*(IMG_V_ - height_) / (a - (IMG_V_ - height_));

		float delta = (float)0.1;

		v_idx_.push_back(IMG_V_ - (int)((a*lower_y) / (float)(b + lower_y)));

		for (float y = lower_y; y < upper_y; y = y + delta) {
			int temp_val = IMG_V_ - (int)((a*y) / (float)(b + y));
			if (temp_val == (int)v_idx_[v_idx_.size() - 1]) {
				continue;
			}
			v_idx_.push_back(temp_val);
			//std::cout << temp_val << " " << y << std::endl;
		}
	}

	// input from parameter file
	frame_count_ = 0;

	// Estimated Lane Width
	max_lw_.resize(height_);
	min_lw_.resize(height_);
	max_lw_d_.resize(width_);
	var_lw_.resize(height_);

	float c = 0.5;
	for (int hh = height_roi_; hh < height_; ++hh) {
		max_lw_[hh] = (int)((MAX_LW_N - MAX_LW_F)*(hh - height_roi_) / (height_ - height_roi_) + MAX_LW_F);
		min_lw_[hh] = (int)((MIN_LW_N - MIN_LW_F)*(hh - height_roi_) / (height_ - height_roi_) + MIN_LW_F);
		var_lw_[hh] = c*max_lw_[hh];
	}

	int w = width_ - 1;
	while (width_ - 1 - w < w) {
		max_lw_d_[w] = (int)(MAX_LW_D*(abs(w - (width_ - 1) / 2.0)) / ((width_ - 1) / 2.0));
		max_lw_d_[width_ - 1 - w] = (int)(MAX_LW_D*(abs(w - (width_ - 1) / 2.0)) / ((width_ - 1) / 2.0));
		w--;
	}

	// initialize 6 lanes
	for (int ii = 0; ii < 6; ++ii) {
		lane_[ii] = new ParticleFilter(height_, height_roi_, width_);
	}

	// loop2 variables
	//frame_n_ = 0;
	//road_val_ = 100;
	//road_sat_ = 100;

}

void LaneDetection::initialize_img(cv::Mat& img_src) {

	// Load image
	img_src_ = img_src;
	img_gray_ = cv::Mat(height_, width_, CV_8UC1);
	if (img_src.channels() == 1 && mode_color_ == true) {
		std::cout << " Warning: input image is in the gray scale. processing in the gray mode" << std::endl;
		mode_color_ = false;
	}

	if (!mode_color_) {	// gray mode
		if (img_src.channels() == 1) {
			img_src.copyTo(img_gray_);
		}
		else {
			cv::cvtColor(img_src, img_gray_, CV_BGR2GRAY);
		}
	}
	else {	// color mode
		cv::Mat hsv;
		cv::cvtColor(img_src, hsv, CV_BGR2HSV);
		//cv::split(hsv, img_hue_, img_sat_, img_val_);
		//cvSmooth(img_val_, img_val_, CV_MEDIAN, 3, 3);*/
		//cv::split()
	}

	// variable initialization
	lm_.resize(0);
	marking_seed_.resize(0);
}

int LaneDetection::FS_driving_lane_detection(int flag_init) {

	// Common Variable Initialization

	// Loop 1 Initialization - if neccessary
	if (flag_init) {
		
		//flag_valid_ = 0;			// 0: initial, 1: left, 2: right, 3: both
		frame_count_ = 0;

		//// revise later
		lane_[0]->initialize_variable();
		lane_[1]->initialize_variable();
		lane_[2]->initialize_variable();
		lane_[3]->initialize_variable();
		lane_[4]->initialize_variable();
		lane_[5]->initialize_variable();

		l_l_ = 0;
		l_r_ = 1;
		l_l2_ = 2;
		l_r2_ = 3;
		l_l3_ = 4;
		l_r3_ = 5;

	}

	if (verbose_FS_lm) {
		display_FS_lm_ = img_src_.clone();
	}

	if (verbose_FS_lm) {
		std::cout << "lm_detect > ";
	}
	lane_marking_detection(false);
	
	if (verbose_FS_lm) {
		std::cout << "seed > ";
	}
	FS_seed_generation(false);
	
	if (verbose_FS_lm) {
		std::cout << "data_collect > ";
	}
	int collect_flag = FS_data_collect();
	
	if (collect_flag == 1) {
		if (verbose_FS_lm) {
			std::cout << "hypothesis > ";
		}
		FS_DL_hypothesis();

		if (verbose_FS_lm) {
			std::cout << "verification > ";
		}
		FS_DL_verification();
	}

	//std::cout << lane_[l_l2_]->valid_g << "    " << lane_[l_l_]->valid_g << "    " << lane_[l_r_]->valid_g << "    " << lane_[l_r2_]->valid_g << std::endl;
	
	if (verbose_FS_lm) {
		cv::Mat img_resize = cv::Mat(height_*0.5, width_*0.5, CV_8UC3);
		cv::resize(display_FS_lm_, img_resize, img_resize.size());
		cv::imshow("Lane marking detection", img_resize);
	}	
	
	if (verbose_FS_lm) {
		std::cout << std::endl;
	}
	
	int valid_num = lane_[l_l2_]->valid_g + lane_[l_l_]->valid_g + lane_[l_r_]->valid_g + lane_[l_r2_]->valid_g;
	if (valid_num >= 2) {	// ToDo : when valid_num == 1
		return 2;
	}
	else {
		return 1;
	}
}

void LaneDetection::FS_lane_detection_g(ParticleFilter* pf, int lane_idx) {

	//float init_vp_y = 242.f;

	//// Lane Marking Detection
	//if (lane_idx == 0) {
	//	// left lane
	//	pf->c_p = cv::Point2f(width_* 0.42f, IMG_V_);
	//	pf->slope = (width_ / 2.f - pf->c_p.x) / (init_vp_y - pf->c_p.y);

	//}
	//else if (lane_idx == 1) {
	//	// right lane
	//	pf->c_p = cv::Point2f(width_* 0.58f, IMG_V_);
	//	pf->slope = (width_ / 2.f - pf->c_p.x) / (init_vp_y - pf->c_p.y);
	//}

	// Lane mark detection
	pf->lane_f_g.resize(0);
	
	for (int n = 0; n < v_idx_.size(); n++) {
		float temp_x = pf->slope*(v_idx_[n] - pf->c_p.y) + pf->c_p.x;
		int w_s = (int)(temp_x - 2 * max_lw_[(int)v_idx_[n]]);
		int w_e = (int)(temp_x + 2 * max_lw_[(int)v_idx_[n]]);

		if (w_e <= 0) continue;
		if (w_s >= width_) continue;
		if (w_s <= 0) w_s = 0;
		if (w_e >= width_) w_e = width_ - 1;
		if (verbose_FS_lm) {
			cv::line(display_FS_lm_, cvPoint(w_s, v_idx_[n]), cvPoint(w_e, v_idx_[n]), CV_RGB(70, 130, 70), 1, 8, 0);
		}
		HV_lm_detection_g(pf, w_s, w_e, v_idx_[n]);
	}

	if (verbose_FS_lm) {
		for (int ii = 0; ii < pf->lane_f_g.size(); ++ii) {
			//std::cout << pf->lane_f_g[ii].str_p.x << " " << pf->lane_f_g[ii].end_p.x << std::endl;
			cv::line(display_FS_lm_, cvPoint(pf->lane_f_g[ii].str_p.x, pf->lane_f_g[ii].str_p.y), cvPoint(pf->lane_f_g[ii].end_p.x, pf->lane_f_g[ii].end_p.y), CV_RGB(0, 255, 0), 2, 8, 0);
		}
	}

	// Lane fitting
	int n_feature = pf->lane_f_g.size();

	if (n_feature < 7) {
		pf->valid_g = 0;
		return;
	}

	int max_iter = 15;
	int iter = 0;
	std::vector<int> inlier, inlier_temp;
	int inlier_type = 0;
	inlier.resize(n_feature);
	inlier_temp.resize(n_feature);
	int n_inlier = 0;
	int n_inlier_temp = 0;
	int flag = 0;
	int rnd_flag = 0;
	int n_rnd_number = std::max(3, (int)(0.4f * n_feature));
	std::vector<int> rnd_idx(n_feature);
	for (int nn = 0; nn < rnd_idx.size(); ++nn) {
		rnd_idx[nn] = nn;
	}


	// Model Parameter
	POLYNOMIAL poly2, poly3;
	polynomial_initialize(poly2, 2);
	polynomial_initialize(poly3, 3);
	
	std::vector<cv::Point2f> points(n_feature);
	std::vector<float> pts_y(n_feature), pts_x(n_feature);
	for (int ii = 0; ii < n_feature; ++ii){
		pts_y[ii] = (float)pf->lane_f_g[ii].cnt_p.y;
	}

	// Initialization
	for (int i = 0; i < n_feature; i++) {
		inlier[i] = 0;
	}
	srand((unsigned)time(NULL));

	while ((iter < max_iter) && (flag < 1)) {

		// Initialization -RANDOM 
		std::random_shuffle(rnd_idx.begin(), rnd_idx.end());

		// Initialization - Variables
		n_inlier_temp = 0;
		for (int i = 0; i < n_feature; i++) {
			inlier_temp[i] = 0;
		}

		// Modeling
		for (int i = 0; i < n_rnd_number; i++) {
			points[i].x = (float)pf->lane_f_g[rnd_idx[i]].cnt_p.x;
			points[i].y = (float)pf->lane_f_g[rnd_idx[i]].cnt_p.y;
		}
		
		// Inlier Finding 
		{
			getPoly3_norm(points, n_rnd_number, poly3);
			fittingLinesWithPoly(pts_y, pts_x, poly3);

			float error = 0.f;
			for (int ii = 0; ii < n_feature; ++ii) {
				error = abs(pts_x[ii] - pf->lane_f_g[ii].cnt_p.x);

				if (error < min_lw_[(int)(pf->lane_f_g[ii].cnt_p.y)]) {
					inlier_temp[ii] = 1;
					n_inlier_temp++;
				}
			}

			if (n_inlier_temp > n_inlier) {
				n_inlier = n_inlier_temp;
				for (int i = 0; i < n_feature; i++) {
					inlier[i] = inlier_temp[i];
				}
				inlier_type = 3;
				// Inlier Sufficiency
				if (n_inlier > 0.9*n_feature) {
					flag = 1;
					break;
				}
			}
		}
		{
			getPoly2_norm(points, n_rnd_number, poly2);
			fittingLinesWithPoly(pts_y, pts_x, poly2);

			float error = 0.f;
			for (int ii = 0; ii < n_feature; ++ii) {
				error = abs(pts_x[ii] - pf->lane_f_g[ii].cnt_p.x);

				if (error < min_lw_[(int)(pf->lane_f_g[ii].cnt_p.y)]) {
					inlier_temp[ii] = 1;
					n_inlier_temp++;
				}
			}

			if (n_inlier_temp > n_inlier) {
				n_inlier = n_inlier_temp;
				for (int i = 0; i < n_feature; i++) {
					inlier[i] = inlier_temp[i];
				}
				inlier_type = 2;
				// Inlier Sufficiency
				if (n_inlier > 0.9*n_feature) {
					flag = 1;
					break;
				}
			}
		}
		iter++;
	}

	if ((n_inlier < 5) || (n_inlier < 0.5*n_feature)) {
		pf->valid_g = 0;
	}

	if (verbose_FS_lm) {
		for (int ii = 0; ii < n_feature; ii++) {
			if (inlier[ii] == 1) {
				//std::cout << pf->lane_f_g[ii].str_p.x << " " << pf->lane_f_g[ii].end_p.x << std::endl;
				cv::line(display_FS_lm_, cvPoint(pf->lane_f_g[ii].str_p.x, pf->lane_f_g[ii].str_p.y), cvPoint(pf->lane_f_g[ii].end_p.x, pf->lane_f_g[ii].end_p.y), CV_RGB(0, 255, 255), 2, 8, 0);
			}
		}
	}

	// Modeling
	std::vector<cv::Point2f> final_inlier;
	for (int ii = 0; ii < n_feature; ++ii) {
		if (inlier[ii] == 1) {
			final_inlier.push_back(pf->lane_f_g[ii].cnt_p);
		}
	}
	getPoly3_norm(final_inlier, final_inlier.size(), poly3);
	getPoly2_norm(final_inlier, final_inlier.size(), poly2);
	float err_poly3 = calculate_fitting_error(final_inlier, poly3);
	float err_poly2 = calculate_fitting_error(final_inlier, poly2);
	
	//// whether fitting into ROI
	//bool isFittedPoly2 = isInsideROI(pf, poly2);
	//bool isFittedPoly3 = isInsideROI(pf, poly3);

	//if (!isFittedPoly2) {
	//	pf->valid_g = 0;
	//	return;
	//}

	if (err_poly3 > 6.0f || err_poly2 > 6.0f) {
		return;
	}
	else {
		pf->valid_g = 1;

		// Data Update and Tracking
		pf->z3_g = poly3;
		pf->z2_g = poly2;

		//printf(" PF \n");
		pf->initialize_filter_g();
		pf->excution(var_lw_, pf->z3_g, pf->z2_g, pf->x3_g, pf->x2_g);

		// roi save
		pf->c_p.x = (pf->x2_g.coeff[1] * (pf->c_p.y - pf->x2_g.avg_p.y) / (pf->x2_g.scale) + pf->x2_g.coeff[0]) * pf->x2_g.scale + pf->x2_g.avg_p.x;
		pf->slope = pf->x2_g.coeff[1];	

	}

	if (verbose_FS_lm) {
		std::vector<float> pts_yy, pts_xx;
		for (int hh = height_; hh > height_roi_; --hh) {
			pts_yy.push_back(hh);
		}
		fittingLinesWithPoly(pts_yy, pts_xx, poly2);
		for (int hh = 0; hh < pts_yy.size(); ++hh) {
			cv::circle(display_FS_lm_, cvPoint(pts_xx[hh], pts_yy[hh]), 1, CV_RGB(250, 0, 0), 1, 8, 0);
		}
		fittingLinesWithPoly(pts_yy, pts_xx, poly3);			
		for (int hh = 0; hh < pts_yy.size(); ++hh) {
			cv::circle(display_FS_lm_, cvPoint(pts_xx[hh], pts_yy[hh]), 1, CV_RGB(0, 0, 250), 1, 8, 0);
		}		
	}
}

inline double LaneDetection::FS_marking_thres(double input) {

	double thres = 0;

	//if (input < 50) {
	//	thres = (int)(input / 10 + 6);
	//}
	//else {
	//	thres = (int)(input / 50 + 10);
	//}
	return input / 5 + 5;

	/*if(input<50){
	thres = (int)(input/10+6);
	}else{
	thres = (int)(input/50+10);
	}
	return thres;*/

}

void LaneDetection::lane_marking_detection(bool verbose){

	lm_.resize(0);

	// lane marking detection
	for (int h = height_roi_; h < height_;) {

		// half size of the filter
		int hf_size = 3 + 9 * (h - height_roi_ + 1) / (height_ - height_roi_);
		
		std::vector<int> scan_line(width_+20);

		// Edge Extraction
		for (int w = hf_size + 1; w < width_ - hf_size - 1; w++) {

			// left edge value, right edge value
			int l_val = 0;
			int r_val = 0;

			for (int i = -hf_size; i<0; i++) {
				l_val = l_val + img_gray_.at<uchar>(h, w + i);
			}
			for (int i = 1; i <= hf_size; i++) {
				r_val = r_val + img_gray_.at<uchar>(h, w + i);
			}
			if (((float)(r_val - l_val) / (float)hf_size)>FS_marking_thres((float)l_val / (float)hf_size)) scan_line[w] = 1; // left edge = 1;
			if (((float)(l_val - r_val) / (float)hf_size)>FS_marking_thres((float)r_val / (float)hf_size)) scan_line[w] = -1; // right edge = -1;
		}

		// Edge Centering
		int e_flag = 0; // edge flag
		for (int w = hf_size + 1; w < width_ - hf_size - 2; w++) {
			if (scan_line[w] == 1) {
				if (e_flag >= 0) {
					e_flag++;
				}
				else {
					scan_line[w - (int)(e_flag / 2.0)] = -10;
					e_flag = 0;
				}
			}
			else if (scan_line[w] == -1) {
				if (e_flag <= 0) {
					e_flag--;
				}
				else {
					scan_line[w + (int)(e_flag / 2.0)] = 10;
					e_flag = 0;
				}
			}
			else {
				if (e_flag > 0) {
					scan_line[w - (int)(e_flag / 2.0)] = 10;
					e_flag = 0;
				}
				else if (e_flag < 0) {
					scan_line[w + (int)(e_flag / 2.0)] = -10;
					e_flag = 0;
				}
			}
		}

		// Extracting Lane Markings - marking flag
		cv::Point2i l_pt, r_pt;
		int m_flag = 0;

		for (int w = hf_size + 1; w < width_ - hf_size - 1; w++) {
			if (scan_line[w] == 10) {
				m_flag = 1;
				l_pt.x = w;
				l_pt.y = h;
			}
			if (m_flag == 1) {
				if (scan_line[w] == -10) {
					m_flag = 2;
					r_pt.x = w;
					r_pt.y = h;
				}
			}
			if (m_flag == 2) {
				if (((r_pt.x - l_pt.x) >= min_lw_[h]) && ((r_pt.x - l_pt.x) <= (max_lw_[h] + max_lw_d_[w]))) {
					
					// lane update
					LANE_MARKING lm_new;
					lm_new.str_p = l_pt;
					lm_new.end_p = r_pt;
					lm_new.cnt_p.x = (int)((l_pt.x + r_pt.x) / 2.0);
					lm_new.cnt_p.y = r_pt.y;
					if (lm_new.cnt_p.x > (int)(width_ / 2)) {
						lm_new.inn_p = l_pt;
					}
					else {
						lm_new.inn_p = r_pt;
					}
					lm_new.size = r_pt.x - l_pt.x;
					lm_.push_back(lm_new);
					w = r_pt.x + 5;
					m_flag = 0;
					if (lm_.size() >= MAX_LANE_MARKING - 1) {
						break;
					}
				}
				m_flag = 0;
			}
		}
		if (lm_.size() >= MAX_LANE_MARKING - 1) {
			break;
		}

		if (h <  350) {
			h += SCAN_INTERVAL1;
		}
		else {
			h += SCAN_INTERVAL2;
		}
	}

	if (verbose) {
		for (int n = 0; n < lm_.size(); n++) {
			cv::line(display_FS_lm_, lm_[n].str_p, lm_[n].end_p, CV_RGB(0, 255, 0), 2, 8, 0);
		}
	}


	
}

void LaneDetection::FS_seed_generation(bool verbose){

	// Initialization

	// STEP 1-1. Generating Seeds: Making a bunch of seeds consisting of lane markings near each others.
	int flag_group = 0;
	int flag_dist = 0;
	for (int ii = 0; ii < lm_.size(); ii++) {
		flag_group = 0;
		for (int jj = marking_seed_.size() - 1; jj >= 0; jj--) {

			flag_dist = FS_dist_ftn1(ii, marking_seed_[jj].index[marking_seed_[jj].index.size() - 1], marking_seed_[jj].cnt_dir);

			if (flag_dist == 1) {
				flag_group = 1;
				marking_seed_[jj].index.push_back(ii);
				if (marking_seed_[jj].cnt_dir < -99) {
					marking_seed_[jj].cnt_dir = slope_ftn(lm_[ii].cnt_p, marking_seed_[jj].cnt_p);
				}
				else {
					marking_seed_[jj].cnt_dir = 0.8*marking_seed_[jj].cnt_dir + 0.2*slope_ftn(lm_[ii].cnt_p, marking_seed_[jj].cnt_p);
				}
				marking_seed_[jj].cnt_p = lm_[ii].cnt_p;

				break;
			}
		}
		if (flag_group == 0) {
			MARKING_SEED seed_new;
			seed_new.flag = 0;
			seed_new.index.resize(0);
			seed_new.index.push_back(ii);
			seed_new.cnt_dir = -100;
			seed_new.cnt_p = lm_[ii].cnt_p;
			marking_seed_.push_back(seed_new);
		}
	}

	if (verbose) {
		cv::Mat img_test_marking_seed = cv::Mat(height_, width_, CV_8UC3);
		for (int ii = 0; ii < marking_seed_.size(); ++ii) {
			int	r = rand() % 200 + 50;
			int	g = rand() % 200 + 50;
			int b = rand() % 200 + 50;
			for (int jj = 0; jj < marking_seed_[ii].index.size(); ++jj) {
				int idx = marking_seed_[ii].index[jj];
				cv::line(img_test_marking_seed, lm_[idx].str_p, lm_[idx].end_p, CV_RGB(r, g, b), 2, 8, 0);
			}
		}
		cv::Mat img_resize = cv::Mat(height_*0.4, width_*0.4, CV_8UC3);
		cv::resize(img_test_marking_seed, img_resize, img_resize.size());
		//cv::imshow("Raw marking seeds", img_resize);
	}

	// STEP 1-2. Seed Validation
	int count_i, count_j;
	float var;
	for (int ii = 0; ii < marking_seed_.size(); ii++) {
		count_i = marking_seed_[ii].index.size();

		// if contained lane marking is less then a certain number
		if (count_i < VALID_SEED_MARKING_NUMBER_THRES) {
			marking_seed_[ii].flag = -1;
			continue;
		}
		if (count_i < 10) {
			float mean = 0.f;
			for (int jj = 0; jj < count_i; jj++) {
				int idx_i = marking_seed_[ii].index[jj];
				mean = mean + lm_[idx_i].size;
			}
			mean = (float)mean / (float)count_i;
			float var = 0.f;
			for (int jj = 0; jj < count_i; jj++) {
				int idx_i = marking_seed_[ii].index[jj];
				var = var + (lm_[idx_i].size - mean)*(lm_[idx_i].size - mean);
			}
			var = var / (float)count_i;

			// if variance is higher, it regarded as invalid
			if (var > 6.0) {
				marking_seed_[ii].flag = -1;
			}
		}
	}

	// STEP 1-3. Seed specification: Getting information of each seeds, position & direction
	std::vector<int> val_seed;

	srand((unsigned)time(NULL));
	int r, g, b;
	for (int ii = 0; ii < marking_seed_.size(); ii++) {
		if (marking_seed_[ii].flag < 0) {
			continue;
		}
		FS_seed_specification(marking_seed_[ii], 1);
		val_seed.push_back(ii);
	}

	if (verbose) {
		cv::Mat img_test_valid_seed = cv::Mat(height_, width_, CV_8UC3);
		for (int ii = 0; ii < val_seed.size(); ++ii) {
			int	r = rand() % 200 + 50;
			int	g = rand() % 200 + 50;
			int b = rand() % 200 + 50;

			MARKING_SEED seed = marking_seed_[val_seed[ii]];
			for (int jj = 0; jj < seed.index.size(); ++jj) {
				int idx = seed.index[jj];
				cv::line(img_test_valid_seed, lm_[idx].str_p, lm_[idx].end_p, CV_RGB(r, g, b), 2, 8, 0);
			}
		}

		cv::Mat img_resize = cv::Mat(height_*0.4, width_*0.4, CV_8UC3);
		cv::resize(img_test_valid_seed, img_resize, img_resize.size());
		//cv::imshow("val marking seeds", img_resize);
	}

	// STEP 2. Seed Growing - Dist_mat Generation
	int n_of_valid_seeds = val_seed.size();
	std::vector<int> trns_stats;
	trns_stats.resize(n_of_valid_seeds, -1);
	cv::Mat dist_mat = cv::Mat(n_of_valid_seeds, n_of_valid_seeds, CV_32FC1);

	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		dist_mat.at<float>(ii, ii) = -1.f;
		for (int jj = ii + 1; jj < n_of_valid_seeds; ++jj) {
			dist_mat.at<float>(ii, jj) = FS_dist_ftn2(val_seed[ii], val_seed[jj]);
			dist_mat.at<float>(jj, ii) = dist_mat.at<float>(ii, jj);
		}
	}

	// STEP 2-1. Low Level Association Process #1 - Head -> Tail
	for (int ii = 0; ii < n_of_valid_seeds; ++ii) {
		int cnct_count = 0;
		int cnct_idx = -1;
		for (int jj = 0; jj < ii; ++jj) {
			if (dist_mat.at<float>(jj, ii) > LOW_LEVEL_ASS_THRES) {
				cnct_count++;
				cnct_idx = jj;
			}
		}
		int valid_flag = 0;
		float temp_max = 0;
		int max_id = -1;

		if (cnct_count == 1) {
			for (int kk = cnct_idx; kk<n_of_valid_seeds; kk++) {
				if (dist_mat.at<float>(cnct_idx, kk) > temp_max) {
					temp_max = dist_mat.at<float>(cnct_idx, kk);
					max_id = kk;
				}
			}
			if (max_id == ii) {
				valid_flag = 1;
			}
		}
		if (valid_flag == 1) {
			//	The only seed which comes down to 'cnct_idx' is 'ii'. Thus, 'cnct_idx' has to be connected to 'ii'.
			MARKING_SEED* seed_dst = &marking_seed_[val_seed[ii]];
			MARKING_SEED* seed_connect = &marking_seed_[val_seed[cnct_idx]];
			count_j = seed_connect->index.size();
			for (int kk = 0; kk < count_j; kk++) {
				seed_dst->index.push_back(seed_connect->index[kk]);
			}
			seed_connect->index.resize(0);
			seed_dst->flag = 1;
			seed_connect->flag = -1;	// seed # which become included in i
			FS_seed_specification(*seed_dst, 0);
			seed_dst->str_dir = seed_connect->str_dir;
			seed_dst->str_p = seed_connect->str_p;
			seed_dst->length = seed_dst->length + seed_connect->length;
			for (int ll = cnct_idx; ll < n_of_valid_seeds; ll++) {
				dist_mat.at<float>(cnct_idx, ll) = 0;
			}
			// remember where the transition happened
			trns_stats[cnct_idx] = ii;
		}
	}

	int temp_val = 0;
	int last_idx = 0;
	// STEP 2-2. Low Level Association Process #2 - Head <- Tail
	for (int ii = n_of_valid_seeds - 1; ii >= 0; ii--) {
		int cnct_count = 0;
		int cnct_idx = -1;
		for (int jj = ii + 1; jj < n_of_valid_seeds; jj++) {
			if (dist_mat.at<float>(ii, jj) > LOW_LEVEL_ASS_THRES) {
				cnct_count++;
				cnct_idx = jj;
			}
		}
		int valid_flag = 0;
		int temp_max = 0;
		int max_id = -1;
		if (cnct_count == 1) {
			for (int kk = 0; kk<cnct_idx; kk++) {
				if (dist_mat.at<float>(kk, cnct_idx) > temp_max) {
					temp_max = dist_mat.at<float>(kk, cnct_idx);
					max_id = kk;
				}
			}
			if (max_id == ii) {
				valid_flag = 1;
			}
		}
		if (valid_flag == 1) {
			// remember where the transition happened
			last_idx = cnct_idx;
			temp_val = trns_stats[last_idx];
			while (temp_val != -1) {
				last_idx = temp_val;
				temp_val = trns_stats[last_idx];
			}
			cnct_idx = last_idx;
			// the only seed coming upto 'cnct_idx' is 'i'.
			MARKING_SEED* seed_dst = &marking_seed_[val_seed[ii]];
			MARKING_SEED* seed_connect = &marking_seed_[val_seed[cnct_idx]];
			count_j = seed_connect->index.size();
			for (int kk = 0; kk < count_j; kk++) {
				seed_dst->index.push_back(seed_connect->index[kk]);
			}
			seed_connect->index.resize(0);
			seed_dst->flag = 1;
			seed_connect->flag = -1;
			FS_seed_specification(*seed_dst, 0);
			seed_dst->end_dir = seed_connect->end_dir;
			seed_dst->end_p = seed_connect->end_p;
			seed_dst->length = seed_dst->length + seed_connect->length;
			for (int ll = 0; ll < cnct_idx; ll++) {
				dist_mat.at<float>(ll, cnct_idx) = 0;
			}
		}
	}

	if (verbose) {
		// major two
		std::vector<int> collect_size;
		for (int ii = 0; ii < marking_seed_.size(); ++ii) {
			if (marking_seed_[ii].flag < 0) {
				continue;
			}
			collect_size.push_back(marking_seed_[ii].index.size());
		}
		std::sort(collect_size.begin(), collect_size.end());
		int thres = std::max(collect_size[std::max(0, (int)(collect_size.size() - 3))], 60);	// parameter setting

		// displaying
		cv::Mat img_test_raw_level_assoc = cv::Mat(height_, width_, CV_8UC3);
		for (int ii = 0; ii < marking_seed_.size(); ++ii) {
			if (marking_seed_[ii].flag < 0) {
				continue;
			}

			int	r = rand() % 200 + 50;
			int	g = rand() % 200 + 50;
			int b = rand() % 200 + 50;

			MARKING_SEED seed = marking_seed_[ii];
			float cont_length = length_ftn(seed.str_p, seed.end_p);
			if (seed.index.size() >= thres && cont_length > 200){		// parameter setting
				for (int jj = 0; jj < seed.index.size(); ++jj) {
					int idx = seed.index[jj];
					cv::line(img_test_raw_level_assoc, lm_[idx].str_p, lm_[idx].end_p, CV_RGB(255, 255, 255), 3, 8, 0);
				}
			}
			else {
				//for (int jj = 0; jj < seed.index.size(); ++jj) {
				//	int idx = seed.index[jj];
				//	cv::line(img_test_raw_level_assoc, lm_[idx].str_p, lm_[idx].end_p, CV_RGB(r, g, b), 1, 8, 0);
				//}
			}
		}
		cv::Mat img_resize = cv::Mat(height_*0.4, width_*0.4, CV_8UC3);
		cv::resize(img_test_raw_level_assoc, img_resize, img_resize.size());

		cv::imshow("Low Level Association", img_resize);
		//cv::waitKey(0);
	}

}

void LaneDetection::FS_seed_specification(MARKING_SEED& marking_seed_curr, int mode) {

	float temp_x = 0;
	float temp_y = 0;

	std::vector<cv::Point2f> points;
	int n_of_lm = marking_seed_curr.index.size();

	for (int ii = 0; ii < n_of_lm; ii++) {
		int idx_lm = marking_seed_curr.index[ii];
		temp_x += (float)lm_[idx_lm].cnt_p.x;
		temp_y += (float)lm_[idx_lm].cnt_p.y;
		points.push_back(lm_[idx_lm].cnt_p);
	}

	POLYNOMIAL poly2;
	polynomial_initialize(poly2, 2);
	getPoly2_norm(points, points.size(), poly2);
	marking_seed_curr.cnt_dir = CV_PI / 2 - atan(poly2.coeff[1]);
	marking_seed_curr.cnt_p.x = (int)(temp_x / n_of_lm);
	marking_seed_curr.cnt_p.y = (int)(temp_y / n_of_lm);

	if (mode == 1) {	// initial seed
		marking_seed_curr.str_p = lm_[marking_seed_curr.index[0]].cnt_p;
		marking_seed_curr.end_p = lm_[marking_seed_curr.index[n_of_lm - 1]].cnt_p;
		marking_seed_curr.length = length_ftn(marking_seed_curr.str_p, marking_seed_curr.end_p);
		if (n_of_lm < VALID_SEED_MARKING_NUMBER_THRES) {
			marking_seed_curr.end_dir = marking_seed_curr.cnt_dir;
			marking_seed_curr.str_dir = marking_seed_curr.cnt_dir;
		}
		else {
			int n_samples = std::max(5, (int)(0.3f*n_of_lm));
			getPoly2_norm(points, n_samples, poly2);
			marking_seed_curr.str_dir = (float)(CV_PI / 2 - atan(poly2.coeff[1]));
			points.resize(0);
			for (int ii = n_of_lm - 1; ii >= n_of_lm - n_samples; ii--) {
				int idx_i = marking_seed_curr.index[ii];
				points.push_back(lm_[idx_i].cnt_p);
			}
			getPoly2_norm(points, n_samples, poly2);
			marking_seed_curr.end_dir = (float)(CV_PI / 2 - atan(poly2.coeff[1]));
		}
	}

	//printf("%d %d / %d %d\n", marking_seed[idx].str_p.x, marking_seed[idx].str_p.y, marking_seed[idx].end_p.x, marking_seed[idx].end_p.y);
	// the lowest point(in human frame) is the start point, vice versa

}

int LaneDetection::FS_data_collect() {

	// major two
	std::vector<int> collect_size;
	for (int ii = 0; ii < marking_seed_.size(); ++ii) {
		if (marking_seed_[ii].flag < 0) {
			continue;
		}
		collect_size.push_back(marking_seed_[ii].index.size());
	}
	std::sort(collect_size.begin(), collect_size.end());
	int thres = std::max(collect_size[std::max(0, (int)(collect_size.size() - 3))], 60);	// parameter setting

	std::vector<int> candiate_idx;

	for (int ii = 0; ii < marking_seed_.size(); ++ii) {

		// Pruning	
		if (marking_seed_[ii].flag < 0) {
			continue;
		}
		float cont_length = length_ftn(marking_seed_[ii].str_p, marking_seed_[ii].end_p);

		if (marking_seed_[ii].index.size() < thres || cont_length < 200) {		// parameter setting
			continue;
		}

		std::vector<cv::Point2f> points;
		int n_of_lm = marking_seed_[ii].index.size();

		// Calculation
		for (int jj = 0; jj < n_of_lm; jj++) {
			int idx_lm = marking_seed_[ii].index[jj];
			points.push_back(lm_[idx_lm].cnt_p);
		}

		POLYNOMIAL poly2, poly3;
		polynomial_initialize(poly2, 2);
		getPoly2_norm(points, n_of_lm, poly2);
		float upper_x = fittingPointWithPoly(IMG_V_, poly2);
		float lower_x = fittingPointWithPoly(height_, poly2);
		float slope = (upper_x - lower_x) / (IMG_V_ - height_);

		std::vector<float> lane_data_sub;
		lane_data_sub.push_back(slope);		// below
		lane_data_sub.push_back(upper_x);// upper	 -- paramter estting
		lane_data_.push_back(lane_data_sub);		
	}

	frame_count_++;
	
	if (lane_data_.size() >= max_n_lane_data_ && frame_count_ >= 8) {
		
		return 1;	// Collection done

	}
	else {
		return 0;
	}
}

inline float dist_points(float x1, float y1, float x2, float y2) {
	return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);

}
int LaneDetection::FS_DL_hypothesis() {

	float dist_thres = 5.f;	// parameter setting
	int n_lane_data = lane_data_.size();
	bool ransacFinish = false;
	group_sets_.resize(0);
	group_avg_.resize(0);

	for (int nn = 0; nn < n_lane_data; ++nn) {
		int gg_idx = -1;
		float nn_x = lane_data_[nn][0];
		float nn_y = lane_data_[nn][1];

		for (int gg = 0; gg < group_sets_.size(); ++gg) {
			float dist = dist_points(nn_x, nn_y / 100.f, group_avg_[gg].x, group_avg_[gg].y / 100.f);
			if (dist < dist_thres) {
				gg_idx = gg;
				break;
			}
		}

		if (gg_idx == -1) {
			std::vector<int> curr_group;
			curr_group.push_back(nn);
			group_sets_.push_back(curr_group);
			group_avg_.push_back(cv::Point2f(nn_x, nn_y));
		}
		else {
			float avg_x = (group_avg_[gg_idx].x * (float)group_sets_[gg_idx].size() + nn_x) / ((float)group_sets_[gg_idx].size() + 1.f);
			float avg_y = (group_avg_[gg_idx].y * (float)group_sets_[gg_idx].size() + nn_y) / ((float)group_sets_[gg_idx].size() + 1.f);
			group_sets_[gg_idx].push_back(nn);
			group_avg_[gg_idx] = cv::Point2f(avg_x, avg_y);
		}
	}

	//for (auto group_gg : group_sets_) {
	//	for (auto idx_ii : group_gg) {
	//		std::cout << lane_data_[idx_ii][0] << " " << lane_data_[idx_ii][1] << std::endl;
	//	}
	//	std::cout << std::endl;
	//}



	//// Multi-Ransac
	//int max_iter = 10;
	//int thres = 200;
	//int iter = 0;

	//int n_lane_data = lane_data_.size();
	//std::vector<int> inlier, inlier_temp;
	//inlier.resize(n_lane_data);
	//inlier_temp.resize(n_lane_data);

	//int n_inlier = 0;
	//int n_inlier_temp = 0;

	//int idx;

	//cv::Point2i cnt_p;
	//cv::Point2i avg_p;
	//int flag = 0;
	//int error = 0;
	//int rnd_flag = 0;

	//// Model Parameter
	//cv::Point2i avg1, avg2;
	//int cnt1 = 0;
	//int cnt2 = 0;

	//while ((iter < max_iter) && (flag < 2)) {

	//	// Initialization
	//	rnd_flag = 0;
	//	while (!rnd_flag) {
	//		idx = rand() % n_lane_data;
	//		if (inlier[idx] == 0) {
	//			rnd_flag = 1;
	//		}
	//	}

	//	//idx = rand()%n_lane_data;
	//	cnt_p = cv::Point2i((int)lane_data_[idx][0], (int)lane_data_[idx][1]);
	//	n_inlier_temp = 0;
	//	for (int i = 0; i < n_lane_data; i++) {
	//		inlier_temp[i] = 0;
	//	}

	//	// Inlier Finding
	//	avg_p = cv::Point2i(0, 0);
	//	error = 0;
	//	for (int i = 0; i < n_lane_data; i++) {
	//		if ((FS_dist_ftn3(lane_data_[i][0], lane_data_[i][1], cnt_p.x, cnt_p.y) < thres) && (inlier[i] == 0)) {
	//			inlier_temp[i] = 1;
	//			avg_p.x = avg_p.x + lane_data_[i][0];
	//			avg_p.y = avg_p.y + lane_data_[i][1];
	//			n_inlier_temp++;
	//		}
	//	}
	//	printf(" > 1. n_inlier_temp = %d\n", n_inlier_temp);
	//	// Inlier Sufficiency
	//	if (n_inlier_temp > 0.25*n_lane_data) {

	//		// Modeling
	//		avg_p.x = avg_p.x / n_inlier_temp;
	//		avg_p.y = avg_p.y / n_inlier_temp;
	//		printf(" > 2. avg_p = %d %d\n", avg_p.x, avg_p.y);
	//		// Error
	//		for (int i = 0; i < n_lane_data; i++) {
	//			if (inlier_temp[i]) {
	//				error = error + (int)FS_dist_ftn3(lane_data_[i][0], lane_data_[i][1], avg_p.x, avg_p.y);
	//			}
	//		}
	//		error = error / n_inlier_temp;

	//		printf(" > 3. error = %d\n", error);
	//		// Error Sufficiency
	//		if (error < 50) {
	//			flag++;
	//			for (int i = 0; i < n_lane_data; i++) {
	//				if (inlier_temp[i] == 1) {
	//					inlier[i] = flag;
	//				}
	//			}
	//			if (flag == 1) {
	//				avg1.x = avg_p.x;
	//				avg1.y = avg_p.y;
	//				cnt1 = n_inlier_temp;
	//			}
	//			else if (flag == 2) {
	//				avg2.x = avg_p.x;
	//				avg2.y = avg_p.y;
	//				cnt2 = n_inlier_temp;
	//			}
	//		}
	//		printf(" > 4. iter = %d, flag = %d\n", iter, flag);
	//	}
	//	iter++;
	//}


	//// 나중에 여기에 주행차선이 하나만 탐지되었을 경우의 코드를 넣기.
	//if (((cnt1 + cnt2) > 0.6*n_lane_data) && (flag == 2)) {
	//	//printf(" >> avg1 = %d \n >> avg2 = %d \n", avg1, avg2);
	//	if (avg1.x < avg2.x) {
	//		lane_[l_l_]->offset[0] = avg1.x;
	//		lane_[l_l_]->offset[1] = avg1.y;
	//		lane_[l_r_]->offset[0] = avg2.x;
	//		lane_[l_r_]->offset[1] = avg2.y;
	//	}
	//	else {
	//		lane_[l_l_]->offset[0] = avg2.x;
	//		lane_[l_l_]->offset[1] = avg2.y;
	//		lane_[l_r_]->offset[0] = avg1.x;
	//		lane_[l_r_]->offset[1] = avg1.y;
	//	}
	//	return 1;
	//}
	//else {
		return 0;
	//}
}

bool cvPointComparater(const cv::Point2f &a, const cv::Point2f &b) {
	return a.y < b.y;
}

int LaneDetection::FS_DL_verification(){

	std::vector<cv::Point2f> valid_pts;
	//int approx_lane_width = ;	// parameter setting

	for (int gg = 0; gg < group_sets_.size(); ++gg) {
		if (group_sets_[gg].size() == 1) {
			continue;
		}
		valid_pts.push_back(group_avg_[gg]);
	}

	std::sort(valid_pts.begin(), valid_pts.end(), cvPointComparater);

	//for (auto aa : valid_pts) {
	//	for (auto bb : valid_pts) {
	//		if (aa == bb) {
	//			continue;
	//		}
	//		std::cout << abs(aa.y - bb.y) << std::endl;
	//	}
	//}
	int n_valid_num = valid_pts.size();
	float min_dist_to_cnt = width_;
	int min_idx = -1;
	float max_gap_between_1_lane = 450.f;	// parameter setting
	float max_gap_between_2_lane = 650.f;	// parameter setting
	float const_dist_to_cnt = 250.f;		// parameter setting

	for (int pp = 0; pp < valid_pts.size(); ++pp) {
		//std::cout << valid_pts[pp].y << std::endl;
		float dist_pp = fabsf(valid_pts[pp].y - (float)IMG_U_);
		if (dist_pp < min_dist_to_cnt) {
			min_dist_to_cnt = dist_pp;
			min_idx = pp;
		}
	}
	
	bool b_l1 = false;
	bool b_r1 = false;
	bool b_l2 = false;
	bool b_r2 = false;

	if(min_dist_to_cnt < const_dist_to_cnt){
		
		// right lane 1 - reference
		if (valid_pts[min_idx].y > (float)IMG_U_) {
			
			lane_[l_r_]->slope = valid_pts[min_idx].x;
			lane_[l_r_]->c_p = cv::Point2f(valid_pts[min_idx].y, IMG_V_);
			b_r1 = true;

			if (min_idx + 1 < n_valid_num) {
				if (abs(valid_pts[min_idx + 1].y - valid_pts[min_idx].y) < max_gap_between_1_lane) {
					lane_[l_r2_]->slope = valid_pts[min_idx+1].x;
					lane_[l_r2_]->c_p = cv::Point2f(valid_pts[min_idx + 1].y, IMG_V_);
					b_r2 = true;
				}
			}
			if (min_idx - 1 >= 0) {
				if (abs(valid_pts[min_idx - 1].y - valid_pts[min_idx].y) < max_gap_between_1_lane) {
					lane_[l_l_]->slope = valid_pts[min_idx - 1].x;
					lane_[l_l_]->c_p = cv::Point2f(valid_pts[min_idx - 1].y, IMG_V_);
					b_l1 = true;
					if (min_idx - 2 >= 0) {
						if (abs(valid_pts[min_idx - 2].y - valid_pts[min_idx].y) < max_gap_between_2_lane) {
							lane_[l_l2_]->slope = valid_pts[min_idx - 2].x;
							lane_[l_l2_]->c_p = cv::Point2f(valid_pts[min_idx - 2].y, IMG_V_);
							b_l2 = true;
						}
					}
				}
				else if (abs(valid_pts[min_idx - 1].y - valid_pts[min_idx].y) < max_gap_between_2_lane){
					lane_[l_l2_]->slope = valid_pts[min_idx - 1].x;
					lane_[l_l2_]->c_p = cv::Point2f(valid_pts[min_idx - 1].y, IMG_V_);
					b_l2 = true;
				}
			}
		}
		else { // left lane 1 - reference
			
			lane_[l_l_]->slope = valid_pts[min_idx].x;
			lane_[l_l_]->c_p = cv::Point2f(valid_pts[min_idx].y, IMG_V_);
			b_l1 = true;

			if (min_idx + 1 < n_valid_num) {
				if (abs(valid_pts[min_idx + 1].y - valid_pts[min_idx].y) < max_gap_between_1_lane) {
					lane_[l_r_]->slope = valid_pts[min_idx + 1].x;
					lane_[l_r_]->c_p = cv::Point2f(valid_pts[min_idx + 1].y, IMG_V_);
					b_r1 = true;
					if (min_idx + 2 < n_valid_num) {
						if (abs(valid_pts[min_idx + 2].y - valid_pts[min_idx].y) < max_gap_between_2_lane) {
							lane_[l_r2_]->slope = valid_pts[min_idx + 2].x;
							lane_[l_r2_]->c_p = cv::Point2f(valid_pts[min_idx + 2].y, IMG_V_);
							b_r2 = true;
						}
					}
				}
				else if (abs(valid_pts[min_idx + 1].y - valid_pts[min_idx].y) < max_gap_between_2_lane) {
					lane_[l_r2_]->slope = valid_pts[min_idx + 1].x;
					lane_[l_r2_]->c_p = cv::Point2f(valid_pts[min_idx + 1].y, IMG_V_);
					b_r2 = true;
				}
			}
			if (min_idx - 1 >= 0) {
				if (abs(valid_pts[min_idx - 1].y - valid_pts[min_idx].y) < max_gap_between_1_lane) {
					lane_[l_l2_]->slope = valid_pts[min_idx - 1].x;
					lane_[l_l2_]->c_p = cv::Point2f(valid_pts[min_idx - 1].y, IMG_V_);
					b_l2 = true;
				}
			}
		}	
	}
	else {

		// right lane 2 - reference
		if (valid_pts[min_idx].y > (float)IMG_U_) {

			lane_[l_r2_]->slope = valid_pts[min_idx].x;
			lane_[l_r2_]->c_p = cv::Point2f(valid_pts[min_idx].y, IMG_V_);
			b_r2 = true;

			if (min_idx - 1 >= 0) {
				std::cout << " Logical ERROR " << std::endl;
			}
		}
		else { // left lane 2 - reference
			lane_[l_l2_]->slope = valid_pts[min_idx].x;
			lane_[l_l2_]->c_p = cv::Point2f(valid_pts[min_idx].y, IMG_V_);
			b_l2 = true;

			if (min_idx + 1 < n_valid_num) {
				std::cout << " Logical ERROR " << std::endl;
			}
		}
	}

	// Driving Lane mark detection - debug
	if (b_l1) {
		FS_lane_detection_g(lane_[l_l_], l_l_);
	}
	if (b_r1) {
		FS_lane_detection_g(lane_[l_r_], l_r_);
	}
	if (b_l2) {
		FS_lane_detection_g(lane_[l_l2_], l_l2_);
	}
	if (b_r2) {
		FS_lane_detection_g(lane_[l_r2_], l_r2_);
	}
	
	return 0;
}
int LaneDetection::FS_dist_ftn1(int s_i, int s_j, double slope) {

	// For Seed Generation

	double value = 0;
	double slope_new = slope_ftn(lm_[s_i].cnt_p, lm_[s_j].cnt_p);
	cv::Point2f i, j;
	i = lm_[s_i].cnt_p;
	j = lm_[s_j].cnt_p;
	value = (i.x - j.x)*(i.x - j.x) + (i.y - j.y)*(i.y - j.y);


	if ((lm_[s_i].str_p.x > lm_[s_j].end_p.x) || (lm_[s_i].end_p.x < lm_[s_j].str_p.x)) {
		//printf(">> location err (%d,%d) (%d,%d) \n", lm[s_i].str_p.x, lm[s_i].end_p.x,lm[s_j].str_p.x, lm[s_j].end_p.x);
		return 0;
	}

	//printf("  >> slope : %.3f, diff : %.3f, location : (%d,%d) (%d,%d)", slope, abs(slope-slope_new),  lm[s_i].str_p.x, lm[s_i].end_p.x,lm[s_j].str_p.x, lm[s_j].end_p.x);

	if (value < SEED_MARKING_DIST_THRES) {
		if (slope <= -99) {
			//printf(">> initial\n");
			return 1;
		}
		if ((value>50) && (abs(slope - slope_new) > 1.1)) {
			return 0;
		}
		if (abs(slope - slope_new) < 0.8) {
			//printf(">> slope %.3f\n",abs(slope-slope_new));
			return 1;
		}
		if ((lm_[s_i].cnt_p.x <= lm_[s_j].end_p.x) && (lm_[s_i].cnt_p.x >= lm_[s_j].str_p.x)) {
			//printf(">> location\n");
			return 1;
		}
	}
	return 0;
}
float LaneDetection::FS_dist_ftn2(int i, int j) {

	// For Low level Association

	if (marking_seed_[i].end_p.y > marking_seed_[j].str_p.y) {
		return 0;
	}

	// Rough Verification
	std::vector<float> slp;
	slp.resize(7);
	slp[0] = marking_seed_[i].cnt_dir;
	slp[1] = marking_seed_[j].cnt_dir;
	if ((abs(slp[0] - slp[1]) > 0.5) && (abs(abs(slp[0] - slp[1]) - 3.141592) < 2.641592)) {
		return 0;
	}
	slp[2] = slope_ftn(marking_seed_[i].cnt_p, marking_seed_[j].cnt_p);
	slp[3] = slope_ftn(marking_seed_[i].str_p, marking_seed_[j].str_p);
	slp[4] = slope_ftn(marking_seed_[i].str_p, marking_seed_[j].end_p);
	slp[5] = slope_ftn(marking_seed_[i].end_p, marking_seed_[j].str_p);
	slp[6] = slope_ftn(marking_seed_[i].end_p, marking_seed_[j].end_p);

	// slope variance check
	float slp_mean = (slp[0] + slp[1] + slp[2] + slp[3] + slp[4] + slp[5] + slp[6]) / 7;
	float temp = 0;
	for (int i = 0; i < 7; i++) {
		temp += (slp[i] - slp_mean)*(slp[i] - slp_mean);
	}
	float slp_var = temp / 7;
	if (slp_var > 0.5) {
		return 0;
	}

	// distance ftn between two seeds	
	float sig = 0.25;
	float diff1, diff2;
	diff1 = slp[0] - slp[2];
	diff2 = slp[1] - slp[2];
	// it should be that 1 < 3 < 2 or 2 < 3 < 1
	if (((abs(diff1) + abs(diff2)) > 0.2) && (diff1*diff2 > 0)) {
		return 0;
	}
	if (abs(diff1) > 1.570796) {
		diff1 = abs(diff1 - 3.141592);
	}
	if (abs(diff2) > 1.570796) {
		diff2 = abs(diff2 - 3.141592);
	}

	return (float)(exp(-(diff1)*(diff1) / sig*sig) + exp(-(diff2)*(diff2) / sig*sig));
}
int LaneDetection::FS_dist_ftn3(int p1, int p2, int q1, int q2) {

	return abs(p1 - q1) + abs(p2 - q2);
}

float LaneDetection::slope_ftn(cv::Point2f pos1, cv::Point2f pos2) {

	cv::Point2f temp_pos;
	if (pos1.y > pos2.y) {
		temp_pos = pos1;
		pos1 = pos2;
		pos2 = temp_pos;
	}
	return (float)(acos((double)((pos2.x - pos1.x) / sqrt((float)((pos1.x - pos2.x)*(pos1.x - pos2.x) + (pos1.y - pos2.y)*(pos1.y - pos2.y))))));
}
float LaneDetection::length_ftn(cv::Point2f str_p, cv::Point2f end_p) {

	return sqrt((float)(str_p.x - end_p.x)*(str_p.x - end_p.x) + (float)(str_p.y - end_p.y)*(str_p.y - end_p.y));

}

int LaneDetection::HG_HV_lane_detection(cv::Mat& srcImg, int flag_init) {

	initialize_img(srcImg);

	if (verbose_HV_lm) {
		display_HV_lm_ = img_src_.clone();
	}
	
	// Initial Homography - Adjacent Lane Estimation
	int count = lane_[l_l2_]->valid_g + lane_[l_l_]->valid_g + lane_[l_r_]->valid_g + lane_[l_r2_]->valid_g;
	if (count >= 2) {
		HG_homography_init();
	}
	
	if (lane_[l_l2_]->valid_g == 1) {
		HG_Active_update(lane_[l_l2_]);
	}else {
		HG_Passive_update(lane_[l_l2_]);
	}
	
	if (lane_[l_l_]->valid_g == 1) {
		HG_Active_update(lane_[l_l_]);
	}
	else {
		HG_Passive_update(lane_[l_l_]);
	}

	if (lane_[l_r_]->valid_g == 1) {
		HG_Active_update(lane_[l_r_]);
	}
	else {
		HG_Passive_update(lane_[l_r_]);
	}

	if (lane_[l_r2_]->valid_g == 1) {
		HG_Active_update(lane_[l_r2_]);
	}
	else {
		HG_Passive_update(lane_[l_r2_]);
	}

	if (verbose_HV_lm) {
		std::cout << "l1_new: ";
	}
	HV_DL_detection_new(lane_[l_l_]);
	if (verbose_HV_lm) {
		std::cout << std::endl << "r1_new: ";
	}
	HV_DL_detection_new(lane_[l_r_]);
	if (verbose_HV_lm) {
		std::cout << std::endl << "l2_new: ";
	}
	HV_DL_detection_new(lane_[l_l2_]);
	if (verbose_HV_lm) {
		std::cout << std::endl << "r2_new: ";
	}
	HV_DL_detection_new(lane_[l_r2_]);
	if (verbose_HV_lm) {
		std::cout << std::endl;
	}

	if (verbose_HV_lm) {
		cv::Mat img_resize = cv::Mat(height_*0.5, width_*0.5, CV_8UC3);
		cv::resize(display_HV_lm_, img_resize, img_resize.size());
		cv::imshow("verification", img_resize);
	}

	// Verification
	DL_AL_decision();

	// Failure Test
	bool sum1 = false;
	bool sum2 = false;
	bool sum3 = false;
	bool sum4 = false;
	for (int i = 0; i < 10; i++) {
		if (lane_[l_l2_]->disp_cnt_g[i] == 1) {
			sum1 = true;
		}
		if (lane_[l_l_]->disp_cnt_g[i] == 1) {
			sum2 = true;
		}
		if (lane_[l_r_]->disp_cnt_g[i] == 1) {
			sum3 = true;
		}
		if (lane_[l_r2_]->disp_cnt_g[i] == 1) {
			sum4 = true;

		}
	}
	if ( sum1 + sum2 + sum3 + sum4 < 2) {
		return 1;
	}
	return 2;
}

inline float get_x_coordinate(ParticleFilter *pf, float height){
	return pf->slope * (height - pf->c_p.y) + pf->c_p.x;
}

void LaneDetection::HG_homography_init() {

	// All Processed in openCV Coordinates
	float r1 = (float)0.1;
	float r2 = (float)0.3;
	int height1 = (int)((float)(height_ - height_roi_)*r1 + height_roi_);	// Upper
	int height2 = (int)((float)(height_ - height_roi_)*r2 + height_roi_);	// Below
	
	std::vector<int> lane_idx;
	std::vector<int> lane_valid;
	lane_idx.push_back(l_l2_);
	lane_idx.push_back(l_l_);
	lane_idx.push_back(l_r_);
	lane_idx.push_back(l_r2_);
	lane_valid.push_back(lane_[l_l2_]->valid_g);
	lane_valid.push_back(lane_[l_l_]->valid_g);
	lane_valid.push_back(lane_[l_r_]->valid_g);
	lane_valid.push_back(lane_[l_r2_]->valid_g);

	std::vector<cv::Point2f> srcPts, dstPts;

	//for (int ii = 0; ii < lane_idx.size(); ++ii){
	//	std::cout << lane_valid[ii] << " ";
	//}
	//std::cout << std::endl;

	// 변환하기 전의 점을 입력
	for (int ii = 0; ii < lane_idx.size(); ++ii){
		if (lane_valid[ii] == 1){
			srcPts.push_back(cv::Point2f(get_x_coordinate(lane_[lane_idx[ii]], height1), height1));
			srcPts.push_back(cv::Point2f(get_x_coordinate(lane_[lane_idx[ii]], height2), height2));
			dstPts.push_back(cv::Point2f(get_x_coordinate(lane_[lane_idx[ii]], height1), height1));
			dstPts.push_back(cv::Point2f(get_x_coordinate(lane_[lane_idx[ii]], height1), height2));
		}
	}	

	map_matrix_ = cv::findHomography(dstPts, srcPts, 0);
	map_matrix_.convertTo(map_matrix_, CV_32FC1);

	float dist_lanes = 0.f;
	float num_count = 0.f;
	for (int ii = 0; ii < lane_valid.size(); ++ii){
		for (int jj = ii + 1; jj < lane_valid.size(); ++jj){
			if (lane_valid[ii] == 1 && lane_valid[jj] == 1){
				float pp1 = get_x_coordinate(lane_[lane_idx[ii]], height1);
				float pp2 = get_x_coordinate(lane_[lane_idx[jj]], height1);
				dist_lanes += fabsf(pp1 - pp2);
				num_count += fabsf(ii - jj);
			}
		}
	}

	float lane_width = dist_lanes / num_count;

	for (int ii = 0; ii < lane_idx.size(); ++ii){
		if (lane_valid[ii] == 0){
			bool isFound = false;
			float x_val = 0.f;
			for (int ss = ii-1; ss >= 0; --ss){
				if (lane_valid[ss] == 1){
					x_val = get_x_coordinate(lane_[lane_idx[ss]], height1) + lane_width * (float)abs(ss - ii);
					isFound = true;
					break;
				}
			}
			if (!isFound){
				for (int ss = ii + 1; ss < lane_valid.size(); ++ss){
					if (lane_valid[ss] == 1){
						x_val = get_x_coordinate(lane_[lane_idx[ss]], height1) - lane_width * (float)abs(ss - ii);
						isFound = true;
						break;
					}
				}
			}

			std::vector<cv::Point2f> pts_inputs, pts_outputs;
			pts_inputs.push_back(cv::Point2f(x_val, height1));
			pts_inputs.push_back(cv::Point2f(x_val, height2));
			getTransformedPoints(pts_inputs, map_matrix_, pts_outputs);
			
			float slope_ii = (pts_outputs[0].x - pts_outputs[1].x) / (pts_outputs[0].y - pts_outputs[1].y);

			lane_[lane_idx[ii]]->slope = slope_ii;
			lane_[lane_idx[ii]]->c_p = cv::Point2f(slope_ii*(IMG_V_ - pts_outputs[0].y) + pts_outputs[0].x, IMG_V_);
			lane_[lane_idx[ii]]->valid_g = 1;
		}
	}

	//for (int ii = 0; ii < lane_idx.size(); ++ii){
	//	std::cout << lane_[lane_idx[ii]]->valid_g << " ";
	//}
	//std::cout << std::endl;
}

void LaneDetection::HG_Active_update(ParticleFilter* pf) {
		 
	// THIS IS A PROTOTYPE CODING, WHICH MEANS THIS IS A SUPER-DUPER GARGABE
	float w_x_h_roi = fittingPointWithPoly(height_roi_, pf->x2_g);
	float w_x_h = fittingPointWithPoly(height_, pf->x2_g);
	float w_slope_h_roi = (float)(pf->slope * (height_roi_ - pf->c_p.y) + pf->c_p.x);
	float w_slope_h = (float)(pf->slope * (height_ - pf->c_p.y) + pf->c_p.x);
	
	float fr = (float)0.3;	// Update Ratio
	int thres = 200;
	if (abs(w_x_h_roi - w_slope_h_roi) < thres) {
		cv::Point2f p1(fr*w_slope_h_roi + (1 - fr)*w_x_h_roi, height_roi_);
		cv::Point2f p2(fr*w_slope_h + (1 - fr)*w_x_h, height_);
		pf->slope = (p2.x - p1.x) / (p2.y - p1.y);
		pf->c_p = cv::Point2f(pf->slope*(IMG_V_ - p1.y) + p1.x, IMG_V_);
	}

	// ROI Height Configuraion
	int last_hh = height_roi_;
	for (int hh = height_roi_; hh < height_; ++hh) {

		float xx = pf->slope*(hh - pf->c_p.y) + pf->c_p.x;
		int w_s = (int)(xx - 4 * max_lw_[hh]);
		int w_e = (int)(xx + 4 * max_lw_[hh]);

		if (w_e <= 0) break;
		if (w_s >= width_) break;

		last_hh = hh;
	}
	pf->reduced_roi_h = last_hh;

}
void LaneDetection::HG_Passive_update(ParticleFilter* pf) {

	// ROI Height Configuraion
	int last_hh = height_roi_;
	for (int hh = height_roi_; hh < height_; ++hh) {

		float xx = pf->slope*(hh - pf->c_p.y) + pf->c_p.x;
		int w_s = (int)(xx - 4 * max_lw_[hh]);
		int w_e = (int)(xx + 4 * max_lw_[hh]);

		if (w_e <= 0) break;
		if (w_s >= width_) break;

		last_hh = hh;
	}
	pf->reduced_roi_h = last_hh;
}
void LaneDetection::HV_DL_detection_new(ParticleFilter* pf) {

	// Lane Type Decision
	HV_type_decision(pf);
	HV_lane_detection_g(pf);

	// Final Validity
	pf->final_validity();

}
void LaneDetection::HV_lane_detection_g(ParticleFilter* pf) {

	// Lane mark detection
	pf->lane_f_g.resize(0);
	//std::cout << "feat ";

	std::vector<float> set_xx(v_idx_.size());
	if(pf->lane_type == 3){
		fittingLinesWithPoly(v_idx_, set_xx, pf->x3_g);
	}
	else {
		fittingLinesWithPoly(v_idx_, set_xx, pf->x2_g);
	}
	
	for (int n = 0; n < v_idx_.size(); n++) {
		float temp_x = pf->slope*(v_idx_[n] - pf->c_p.y) + pf->c_p.x; 
		if (pf->x3_g.scale != 1) {	// if this is not the first time
			temp_x = set_xx[n];
		}
		int w_s = (int)(temp_x - 2 * max_lw_[(int)v_idx_[n]]);
		int w_e = (int)(temp_x + 2 * max_lw_[(int)v_idx_[n]]);

		if (w_e <= 0) continue;
		if (w_s >= width_) continue;
		if (w_s <= 0) w_s = 0;
		if (w_e >= width_) w_e = width_ - 1;
		if (verbose_HV_lm) {
			cv::line(display_HV_lm_, cvPoint(w_s, v_idx_[n]), cvPoint(w_e, v_idx_[n]), CV_RGB(70, 130, 70), 1, 8, 0);
		}
		HV_lm_detection_g(pf, w_s, w_e, v_idx_[n]);
	}

	if (verbose_HV_lm) {
		for (int ii = 0; ii < pf->lane_f_g.size(); ++ii) {
			//std::cout << pf->lane_f_g[ii].str_p.x << " " << pf->lane_f_g[ii].end_p.x << std::endl;
			cv::line(display_HV_lm_, cvPoint(pf->lane_f_g[ii].str_p.x, pf->lane_f_g[ii].str_p.y), cvPoint(pf->lane_f_g[ii].end_p.x, pf->lane_f_g[ii].end_p.y), CV_RGB(0, 255, 0), 2, 8, 0);
		}
	}

	if (pf->lane_f_g.size() < 10) {
		if (verbose_HV_lm) {
			std::cout << "Low n_Feat " << pf->lane_f_g.size();
		}
		pf->valid_g = 0;
		return;
	}

	// Lane fitting
	int n_feature = pf->lane_f_g.size();
	int max_iter = 15;
	int iter = 0;
	std::vector<int> inlier, inlier_temp;
	inlier.resize(n_feature);
	inlier_temp.resize(n_feature);
	int inlier_type = 0;
	int n_inlier = 0;
	int n_inlier_temp = 0;
	int flag = 0;
	int rnd_flag = 0;
	int n_rnd_number = std::max(3, (int)(0.3f * n_feature));
	std::vector<int> rnd_idx(n_feature);
	for (int nn = 0; nn < rnd_idx.size(); ++nn) {
		rnd_idx[nn] = nn;
	}


	// Model Parameter
	POLYNOMIAL poly2, poly3;
	polynomial_initialize(poly2, 2);
	polynomial_initialize(poly3, 3);

	std::vector<cv::Point2f> points(n_feature);
	std::vector<float> pts_y(n_feature), pts_x(n_feature);
	for (int ii = 0; ii < n_feature; ++ii){
		pts_y[ii] = (float)pf->lane_f_g[ii].cnt_p.y;
	}

	// Initialization
	for (int i = 0; i < n_feature; i++) {
		inlier[i] = 0;
	}
	srand((unsigned)time(NULL));

	while ((iter < max_iter) && (flag < 1)) {

		// Initialization -RANDOM 
		std::random_shuffle(rnd_idx.begin(), rnd_idx.end());

		// Initialization - Variables
		n_inlier_temp = 0;
		for (int i = 0; i < n_feature; i++) {
			inlier_temp[i] = 0;
		}

		// Modeling
		for (int i = 0; i < n_rnd_number; i++) {
			points[i].x = (float)pf->lane_f_g[rnd_idx[i]].cnt_p.x;
			points[i].y = (float)pf->lane_f_g[rnd_idx[i]].cnt_p.y;
		}

		// Inlier Finding 
		{
			getPoly3_norm(points, n_rnd_number, poly3);
			fittingLinesWithPoly(pts_y, pts_x, poly3);
			float error = 0.f;
			for (int ii = 0; ii < n_feature; ++ii) {
				error = abs(pts_x[ii] - pf->lane_f_g[ii].cnt_p.x);

				if (error < 1.5*min_lw_[(int)(pf->lane_f_g[ii].cnt_p.y)]) {	// parameter setting
					inlier_temp[ii] = 1;
					n_inlier_temp++;
				}
			}

			if (n_inlier_temp > n_inlier) {
				n_inlier = n_inlier_temp;
				inlier_type = 3;
				for (int i = 0; i < n_feature; i++) {
					inlier[i] = inlier_temp[i];
				}
				// Inlier Sufficiency
				if (n_inlier > 0.9*n_feature) {
					flag = 1;
					break;
				}
			}
		}
		{
			getPoly2_norm(points, n_rnd_number, poly2);
			fittingLinesWithPoly(pts_y, pts_x, poly2);
			float error = 0.f;
			for (int ii = 0; ii < n_feature; ++ii) {
				error = abs(pts_x[ii] - pf->lane_f_g[ii].cnt_p.x);

				if (error < 1.5*min_lw_[(int)(pf->lane_f_g[ii].cnt_p.y)]) {	// parameter setting
					inlier_temp[ii] = 1;
					n_inlier_temp++;
				}
			}

			if (n_inlier_temp > n_inlier) {
				n_inlier = n_inlier_temp;
				inlier_type = 2;
				for (int i = 0; i < n_feature; i++) {
					inlier[i] = inlier_temp[i];
				}
				// Inlier Sufficiency
				if (n_inlier > 0.9*n_feature) {
					flag = 1;
					break;
				}
			}
		}
		iter++;
	}

	if (verbose_HV_lm) {
		for (int ii = 0; ii < n_feature; ii++) {
			if (inlier[ii] == 1) {
				//std::cout << pf->lane_f_g[ii].str_p.x << " " << pf->lane_f_g[ii].end_p.x << std::endl;
				cv::line(display_HV_lm_, cvPoint(pf->lane_f_g[ii].str_p.x, pf->lane_f_g[ii].str_p.y), cvPoint(pf->lane_f_g[ii].end_p.x, pf->lane_f_g[ii].end_p.y), CV_RGB(0, 0, 255), 1, 8, 0);
			}
			else{
				cv::line(display_HV_lm_, cvPoint(pf->lane_f_g[ii].str_p.x, pf->lane_f_g[ii].str_p.y), cvPoint(pf->lane_f_g[ii].end_p.x, pf->lane_f_g[ii].end_p.y), CV_RGB(255, 0, 0), 1, 8, 0);
			}
		}
	}

	// Exception Handling
	if ((n_inlier < 5) || (n_inlier < 0.5*n_feature)) {
		if (verbose_HV_lm) {
			std::cout << "Low inlier " << n_inlier << "/" << n_feature << " ";
		}
		pf->valid_g = 0;
		return;
	}

	// Modeling
	std::vector<cv::Point2f> final_inlier;
	for (int ii = 0; ii < n_feature; ++ii) {
		if (inlier[ii] == 1) {
			final_inlier.push_back(pf->lane_f_g[ii].cnt_p);
		}
	}
	getPoly3_norm(final_inlier, final_inlier.size(), poly3);
	getPoly2_norm(final_inlier, final_inlier.size(), poly2);
	float err_poly3 = calculate_fitting_error(final_inlier, poly3);
	float err_poly2 = calculate_fitting_error(final_inlier, poly2);

	bool isFittedPoly = false;
	if (err_poly3 > err_poly2) {
		pf->lane_type = 2;
		isFittedPoly = isInsideROI(pf, poly2);
	}
	else {
		pf->lane_type = 3;
		isFittedPoly = isInsideROI(pf, poly3);
	}


	if (!isFittedPoly) {
		pf->valid_g = 0;
		return;
	}

	//if (isFittedPoly2 && !isFittedPoly3) {
	//	poly3.coeff[2] = 0.f;
	//	poly3.coeff[1] = poly2.coeff[1];
	//	poly3.coeff[0] = poly2.coeff[0];
	//	poly3.avg_p = poly2.avg_p;
	//	poly3.scale = poly2.scale;
	//}

	if (verbose_HV_lm) {
		std::cout << "poly error " << err_poly2 << " " << err_poly3 << " ";
	}

	if (std::min(err_poly3, err_poly2) > 6.0f) {
		pf->valid_g = 0;
	}
	else {
		pf->valid_g = 1;

		// Data Update and Tracking
		pf->z3_g = poly3;
		pf->z2_g = poly2;
		pf->initialize_filter_g();
		pf->excution(var_lw_, pf->z3_g, pf->z2_g, pf->x3_g, pf->x2_g);

	}

	if (verbose_HV_lm) {
		std::vector<float> pts_yy, pts_x_x3, pts_x_x2, pts_x_z3, pts_x_z2;
		for (int hh = height_; hh > height_roi_; --hh) {
			pts_yy.push_back(hh);
		}

		if (pf->lane_type == 3) {
			fittingLinesWithPoly(pts_yy, pts_x_x3, pf->x3_g);
			fittingLinesWithPoly(pts_yy, pts_x_z3, pf->z3_g);
			for (int hh = 0; hh < pts_yy.size(); ++hh) {
				cv::circle(display_HV_lm_, cvPoint(pts_x_x3[hh], pts_yy[hh]), 1, CV_RGB(250, 0, 0), 2, 8, 0);
			}
			for (int hh = 0; hh < pts_yy.size(); ++hh) {
				cv::circle(display_HV_lm_, cvPoint(pts_x_z3[hh], pts_yy[hh]), 1, CV_RGB(0, 0, 250), 1, 8, 0);
			}

		}
		else {
			fittingLinesWithPoly(pts_yy, pts_x_x2, pf->x2_g);
			fittingLinesWithPoly(pts_yy, pts_x_z2, pf->z2_g);
			for (int hh = 0; hh < pts_yy.size(); ++hh) {
				cv::circle(display_HV_lm_, cvPoint(pts_x_x2[hh], pts_yy[hh]), 1, CV_RGB(250, 0, 0), 2, 8, 0);
			}
			for (int hh = 0; hh < pts_yy.size(); ++hh) {
				cv::circle(display_HV_lm_, cvPoint(pts_x_z2[hh], pts_yy[hh]), 1, CV_RGB(0, 0, 250), 1, 8, 0);
			}
		}
	}

}
void LaneDetection::HV_type_decision(ParticleFilter* pf) {

	// gray
	pf->color = 0;
}
int LaneDetection::HV_lm_detection_g(ParticleFilter* pf, int w_s, int w_e, int h) {

	int hf_size = 3 + 9 * (h - height_roi_ + 1) / (height_ - height_roi_);
	std::vector<int> scan_line(width_ + 30);

	if (w_s + hf_size + min_lw_[h] >= w_e - hf_size) {
		return 0;
	}

	// Edge Extraction
	for (int w = w_s + hf_size; w < w_e - hf_size - 1; w++) {

		// left edge value, right edge value
		int l_val = 0;
		int r_val = 0;

		for (int i = -hf_size; i<0; i++) {
			l_val = l_val + img_gray_.at<uchar>(h, w + i);
		}
		for (int i = 1; i <= hf_size; i++) {
			r_val = r_val + img_gray_.at<uchar>(h, w + i);
		}
		if (((float)(r_val - l_val) / (float)hf_size)>FS_marking_thres((float)l_val / (float)hf_size)) scan_line[w] = 1; // left edge = 1;
		if (((float)(l_val - r_val) / (float)hf_size)>FS_marking_thres((float)r_val / (float)hf_size)) scan_line[w] = -1; // right edge = -1;					
	}

	// Edge Centering
	int e_flag = 0;
	for (int w = w_s + hf_size; w < w_e - hf_size - 1; w++) {
		if (scan_line[w] == 1) {
			if (e_flag >= 0) {
				e_flag++;
			}
			else {
				scan_line[w - (int)(e_flag / 2.0)] = -10;
				e_flag = 0;
			}
		}
		else if (scan_line[w] == -1) {
			if (e_flag <= 0) {
				e_flag--;
			}
			else {
				scan_line[w + (int)(e_flag / 2.0)] = 10;
				e_flag = 0;
			}
		}
		else {
			if (e_flag > 0) {
				scan_line[w - (int)(e_flag / 2.0)] = 10;
				e_flag = 0;
			}
			else if (e_flag < 0) {
				scan_line[w + (int)(e_flag / 2.0)] = -10;
				e_flag = 0;
			}
		}
	}

	// Extracting Lane Markings - marking flag
	cv::Point2i l_pt, r_pt;
	int m_flag = 0;

	for (int w = w_s + hf_size; w < w_e - hf_size - 1; w++) {
		if (scan_line[w] == 10) {
			m_flag = 1;
			l_pt.x = w;
			l_pt.y = h;
		}
		if (m_flag == 1) {
			if (scan_line[w] == -10) {
				m_flag = 2;
				r_pt.x = w;
				r_pt.y = h;
			}
		}
		if (m_flag == 2) {
			if (((r_pt.x - l_pt.x) >= min_lw_[h]) && ((r_pt.x - l_pt.x) <= (max_lw_[h] + max_lw_d_[w]))) {

				// lane update
				LANE_MARKING lm_new;
				lm_new.str_p = l_pt;
				lm_new.end_p = r_pt;
				lm_new.cnt_p.x = (int)((l_pt.x + r_pt.x) / 2.0);
				lm_new.cnt_p.y = r_pt.y;

				if (lm_new.cnt_p.x > (int)(width_ / 2)) {
					lm_new.inn_p = l_pt;
				}
				else {
					lm_new.inn_p = r_pt;
				}
				lm_new.size = r_pt.x - l_pt.x;
				pf->lane_f_g.push_back(lm_new);
				w = r_pt.x + 4;
				m_flag = 0;

			}
		}
	}

	return 0;
}
double LaneDetection::HV_marking_thres(double input) {

	double thres = 0;

	/*if(input<50){
	thres = (int)(input/10+6);
	}else{
	thres = (int)(input/50+10);
	}
	return thres;*/
	return 7;

}

int LaneDetection::DL_AL_decision() {

	int offset_l = lane_[l_l_]->slope * (height_ - lane_[l_l_]->c_p.y) + lane_[l_l_]->c_p.x;
	int offset_r = lane_[l_r_]->slope * (height_ - lane_[l_r_]->c_p.y) + lane_[l_r_]->c_p.x;
	
	int temp;

	if ((offset_l < IMG_U_) && (offset_r > IMG_U_)) {
		return 0;
	}
	else if ((offset_l > IMG_U_) && (offset_r > IMG_U_)) {

		printf(" >> changing to left lane %d %d -> ", offset_l, offset_r);

		// changing to left lane
		//l_r3 => 삭제		
		lane_[l_r3_]->initialize_variable();
		temp = l_r3_;

		l_r3_ = l_r2_;
		l_r2_ = l_r_;
		l_r_ = l_l_;
		l_l_ = l_l2_;
		l_l2_ = l_l3_;
		l_l3_ = temp;

		return 1;

	}
	else if ((offset_l < IMG_U_) && (offset_r < IMG_U_)) {
		// changing to right lane
		lane_[l_l3_]->initialize_variable();
		temp = l_l3_;

		l_l3_ = l_l2_;
		l_l2_ = l_l_;
		l_l_ = l_r_;
		l_r_ = l_r2_;
		l_r2_ = l_r3_;
		l_r3_ = temp;

		printf(" >> changing to right lane \n");
		return 1;
	}
	else {
		return 1;
	}

	return 0;
}
//double LaneDetection::slope_ftn(CvPoint pos1, CvPoint pos2){
//
//	CvPoint temp_pos;
//	if(pos1.y > pos2.y){
//		temp_pos = pos1;
//		pos1 = pos2;
//		pos2 = temp_pos;
//	}	
//	return acos((double)((pos2.x-pos1.x)/sqrt((float)((pos1.x-pos2.x)*(pos1.x-pos2.x)+(pos1.y-pos2.y)*(pos1.y-pos2.y)))));	
//}


bool LaneDetection::isInsideROI(ParticleFilter* pf, POLYNOMIAL poly) {

	{
		// h_below
		int h_below = pf->reduced_roi_h;
		int w_below_cnt = pf->slope*(h_below - pf->c_p.y) + pf->c_p.x;
		int w_s = (int)(w_below_cnt - 4 * max_lw_[h_below]);
		int w_e = (int)(w_below_cnt + 4 * max_lw_[h_below]);
		int x_below = fittingPointWithPoly(h_below, poly);
		if ((x_below < w_s) || (x_below > w_e)) {
			return false;
		}
	}
	
	{	
		// h_above
		int h_above = height_roi_;
		int w_above_cnt = pf->slope*(h_above - pf->c_p.y) + pf->c_p.x;
		int w_s = (int)(w_above_cnt - 4 * max_lw_[h_above]);
		int w_e = (int)(w_above_cnt + 4 * max_lw_[h_above]);
		int x_above = fittingPointWithPoly(h_above, poly);
		if ((x_above < w_s) || (x_above > w_e)) {
			return false;
		}
	}
	return true;
}
void LaneDetection::displaying() {


	if (verbose_display_final == false) {
		return;
	}

	display_final_ = img_src_.clone();
	for (int ii = 0; ii < 6; ++ii) {

		if (lane_[ii]->disp_g == 0) {
			continue;
		}

		std::vector<float> pts_yy, pts_x_x;
		for (int hh = height_roi_; hh < lane_[ii]->reduced_roi_h; ++hh) {
			pts_yy.push_back(hh);
		}

		if (lane_[ii]->lane_type == 2) {
			fittingLinesWithPoly(pts_yy, pts_x_x, lane_[ii]->x2_g);
		}else{
			fittingLinesWithPoly(pts_yy, pts_x_x, lane_[ii]->x3_g);
		}

		for (int hh = 0; hh < pts_yy.size(); ++hh) {
			cv::circle(display_final_, cvPoint(pts_x_x[hh], pts_yy[hh]), 1, CV_RGB(0, 250, 0), 2, 8, 0);
		}
		//for (int hh = 0; hh < pts_yy.size(); ++hh) {
		//	cv::circle(display_final_, cvPoint(pts_x_x2[hh], pts_yy[hh]), 1, CV_RGB(250, 0, 0), 1, 8, 0);
		//}
	}

	cv::Mat img_resize = cv::Mat(height_*0.5, width_*0.5, CV_8UC3);
	cv::resize(display_final_, img_resize, img_resize.size());
	cv::imshow("Display Final", img_resize);


}

void LaneDetection::writingToOutputFile(Json::Value& currQuery, std::ofstream& output_file, float run_time) {

	{
		Json::Value event_;
		Json::Value lanes(Json::arrayValue); 
		Json::Value lane_l2(Json::arrayValue);
		Json::Value lane_l(Json::arrayValue);
		Json::Value lane_r(Json::arrayValue);
		Json::Value lane_r2(Json::arrayValue);
		
		int n_h_samples = currQuery["h_samples"].size();
		std::vector<float> h_samps;
		lane_l2.resize(n_h_samples);
		lane_l.resize(n_h_samples);
		lane_r.resize(n_h_samples);
		lane_r2.resize(n_h_samples);
		
		for (int ii = 0; ii < n_h_samples; ++ii) {
			lane_l2[ii] = -2;
			lane_l[ii] = -2;
			lane_r[ii] = -2;
			lane_r2[ii] = -2;
			h_samps.push_back((float)currQuery["h_samples"][ii].asFloat());
		}

		std::vector<float> pts_x_l2, pts_x_l, pts_x_r, pts_x_r2;
		if (lane_[l_l2_]->lane_type == 3) {
			fittingLinesWithPoly(h_samps, pts_x_l2, lane_[l_l2_]->x3_g);
		}
		else {
			fittingLinesWithPoly(h_samps, pts_x_l2, lane_[l_l2_]->x2_g);
		}
		if (lane_[l_l_]->lane_type == 3) {
			fittingLinesWithPoly(h_samps, pts_x_l, lane_[l_l_]->x3_g);
		}
		else {
			fittingLinesWithPoly(h_samps, pts_x_l, lane_[l_l_]->x2_g);
		}
		if (lane_[l_r_]->lane_type == 3) {
			fittingLinesWithPoly(h_samps, pts_x_r, lane_[l_r_]->x3_g);
		}
		else {
			fittingLinesWithPoly(h_samps, pts_x_r, lane_[l_r_]->x2_g);
		}
		if (lane_[l_r2_]->lane_type == 3) {
			fittingLinesWithPoly(h_samps, pts_x_r2, lane_[l_r2_]->x3_g);
		}
		else {
			fittingLinesWithPoly(h_samps, pts_x_r2, lane_[l_r2_]->x2_g);
		}

		for (int ii = 0; ii < n_h_samples; ++ii) {
			if (h_samps[ii] < 240) {
				continue;
			}
			float val = pts_x_l2[ii];
			if (val <= 0 || val > width_) {
				break;
			}
			lane_l2[ii] = val;
		}

		for (int ii = 0; ii < n_h_samples; ++ii) {
			if (h_samps[ii] < 240) {
				continue;
			}
			float val = pts_x_l[ii];
			if (val <= 0 || val > width_) {
				break;
			}
			lane_l[ii] = val;
		}

		for (int ii = 0; ii < n_h_samples; ++ii) {
			if (h_samps[ii] < 240) {
				continue;
			}
			float val = pts_x_r[ii];
			if (val <= 0 || val > width_) {
				break;
			}
			lane_r[ii] = val;
		}

		for (int ii = 0; ii < n_h_samples; ++ii) {
			if (h_samps[ii] < 240) {
				continue;
			}
			float val = pts_x_r2[ii];
			if (val <= 0 || val > width_) {
				break;
			}
			lane_r2[ii] = val;
		}

		lanes.append(lane_l2);
		lanes.append(lane_l);
		lanes.append(lane_r);
		lanes.append(lane_r2);

		event_["h_samples"] = currQuery["h_samples"];
		event_["lanes"] = lanes;
		event_["run_time"] = run_time;
		event_["raw_file"] = currQuery["raw_file"];

		Json::StyledStreamWriter writerr("");
		writerr.write(output_file, event_);
	}
}