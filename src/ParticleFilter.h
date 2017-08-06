#pragma once
#include "util.h"

class ParticleFilter{

  public:

	  ParticleFilter(int height, int height_roi, int width) {
		  height_ = height;
		  width_ = width;
		  height_roi_ = height_roi;
		  IMG_U_ = (int)(width_ / 2.f);
		  IMG_V_ = (int)(height_ / 2.f);
		  n_particle_ = 200;

	  }
	  ~ParticleFilter(){}

	  void initialize_variable();
	  void initialize_filter_g();
	  void excution(std::vector<float>& var_lw, POLYNOMIAL& z_, POLYNOMIAL& z2_, POLYNOMIAL& x_, POLYNOMIAL& x2_);
	  void final_validity();
	  	  
	  float randn();
	  void quicksort(float temp_array[], int l, int r);

	  // FS_DL
	  //int offset[2];		// Offset(@IMG_height, ROI_height), only in FS_DL_Detection
	  int flag;	
	  int init;
	  int lane_type;

	  // Particle Filter
	  std::vector<cv::Point2i> x_Update;	// Nomalized Particles
	  std::vector<cv::Point2i> x_P;		// Weighted Particles	
	  std::vector<float> x_P_W;
	  std::vector<float> x_P_W_temp;
	  	  
	  //std::vector<float> z3_g;			// Observations 3nd polynomial
	  //std::vector<float> z2_g;			// Observations 2nd polynomial
	  //std::vector<float> x3_g;			// State 3rd polynomial
	  //std::vector<float> x2_g;			// State 2rd polynomial

	  POLYNOMIAL z3_g;			// Observations 3nd polynomial
	  POLYNOMIAL z2_g;			// Observations 2nd polynomial
	  POLYNOMIAL x3_g;			// State 3rd polynomial
	  POLYNOMIAL x2_g;			// State 2rd polynomial

	  // Lane Marking Detection
	  std::vector<LANE_MARKING> lane_f_g;

	  // ROI Generation
	  float slope;
	  cv::Point2f c_p;
	  int reduced_roi_h;

	  // Properties
	  int init_g;
	  int init_b;
	  int init_y;

	  int color; // 0: grey, 1: blue, 2: yellow
	  int type;  // 0: single, 1: double
	  //int hue_y;
	  //int hue_b;
	  //int sat_b;
	  int valid_g;
	  //int valid_b;
	  //int valid_y;

	  std::vector<int> disp_cnt_g;
	  //int* disp_cnt_b;
	  //int* disp_cnt_y;
	  int disp_g;		// Final validity measure
	  //int disp_b;
	  //int disp_y;
	  
	  int height_;
	  int width_;
	  int height_roi_;
	  int IMG_U_;
	  int IMG_V_;
	  int n_particle_;

  private:


};