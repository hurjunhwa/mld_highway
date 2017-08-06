#pragma once
#include "ParticleFilter.h"
#include <json/json.h>
#include <string>
#include <fstream>

struct MARKING_SEED {
	std::vector<int> index;
	int flag;	// if used 1, otherwise 0
	float cnt_dir;
	float str_dir;
	float end_dir;
	float length;
	cv::Point2f cnt_p;
	cv::Point2f str_p;
	cv::Point2f end_p;
};


class LaneDetection {

public:

	LaneDetection() {
	}
	~LaneDetection() {
	}

	void initialize_variable(cv::Mat& img_src, bool mode_color, float ratio_roi);
	void initialize_img(cv::Mat& img_src);

	// full-searching
	int FS_driving_lane_detection(int loop1_init);
	void FS_lane_detection_g(ParticleFilter* pf, int lane_idx);

	void lane_marking_detection(bool verbose);
	inline double FS_marking_thres(double input);
	void FS_seed_generation(bool verbose);
	void FS_seed_specification(MARKING_SEED& marking_seed_curr, int mode);

	int FS_dist_ftn1(int i, int sj, double slope);
	float FS_dist_ftn2(int idx1, int idx2);
	int FS_dist_ftn3(int i, int j, int s_i, int s_j);
	float slope_ftn(cv::Point2f i, cv::Point2f j);
	float length_ftn(cv::Point2f str_p, cv::Point2f end_p);
		
	int FS_data_collect();
	int FS_DL_hypothesis();
	
	//int FS_DL_verification(IplImage*);
	//int RD_value_extraction(IplImage*);

	int HG_HV_lane_detection(cv::Mat& srcImg, int flag_init);
	void HG_homography_init();
	void HG_Active_update(ParticleFilter*);
	void HG_Passive_update(ParticleFilter*);
	void HV_detection(IplImage*, ParticleFilter*);
	void HV_DL_detection_new(ParticleFilter*);
	void HV_type_decision(ParticleFilter*);
	void HV_lane_detection_g(ParticleFilter*);
	//void HV_lane_detection_b(IplImage*, ParticleFilter*);
	//void HV_lane_detection_y(IplImage*, ParticleFilter*);
	int HV_lm_detection_g(ParticleFilter*, int, int, int);
	//int HV_lm_detection_b(IplImage*, ParticleFilter*, int, int, int);
	//int HV_lm_detection_y(IplImage*, ParticleFilter*, int, int, int);
	double HV_marking_thres(double input);
	bool isInsideROI(ParticleFilter* pf, POLYNOMIAL poly);
	int DL_AL_decision();
	int FS_DL_verification();

	void displaying();
	void writingToOutputFile(Json::Value& currQuery, std::ofstream& output_file, float run_time);
	
	

private:

	// Image
	int width_;
	int height_;
	int height_roi_;
	bool mode_color_;
	cv::Mat img_src_;
	cv::Mat img_gray_;
	cv::Mat img_hue_;
	cv::Mat img_val_;
	cv::Mat img_sat_;

	cv::Mat map_matrix_;

	// Vanishing point
	int IMG_U_;
	int IMG_V_;

	// Lane marking variable
	std::vector<int> max_lw_;
	std::vector<int> min_lw_;
	std::vector<int> max_lw_d_;
	std::vector<LANE_MARKING> lm_;
	std::vector<MARKING_SEED> marking_seed_;
	std::vector<float> v_idx_;

	// particle filter
	ParticleFilter* lane_[6];
	std::vector<float> var_lw_; // variance

	// Lane Marking Grouping 
	
	int n_of_valid_seeds_;
	//int n_of_markings_;


	// Lane Index Assignment
	int l_l_;
	int l_r_;
	int l_l2_;
	int l_r2_;
	int l_l3_;
	int l_r3_;

	// Loop 1 variables
	int frame_count_;			// parameter setting
	int max_n_lane_data_ = 10;
	std::vector<std::vector<float> > lane_data_;
	std::vector<std::vector<int> > group_sets_;
	std::vector<cv::Point2f> group_avg_;
	int flag_valid_;

	//// Loop 2 variables
	//int frame_n_;	// for road color update
	//int road_hue_;
	//int road_sat_;
	//int road_val_;

	// verbose
	bool verbose_FS_lm = true;
	cv::Mat display_FS_lm_;
	bool verbose_HV_lm = true;
	cv::Mat display_HV_lm_;
	bool verbose_display_final = true;
	cv::Mat display_final_;
};