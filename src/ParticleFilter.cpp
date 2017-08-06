#pragma once
#include "ParticleFilter.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>


void ParticleFilter::initialize_variable() {

	x_Update.resize(n_particle_);
	x_P.resize(n_particle_);
	x_P_W.resize(n_particle_);
	x_P_W_temp.resize(n_particle_);

	polynomial_initialize(z3_g, 3);
	polynomial_initialize(x3_g, 3);
	polynomial_initialize(z2_g, 2);
	polynomial_initialize(x2_g, 2);
	
	init = 0;
	flag = 0;
	lane_type = 0;

	// lane validity
	init_g = 0;
	valid_g = 0;
	disp_g = 0;
	
	// DIsplay DB (Validity Count)
	disp_cnt_g.resize(10);

	for (int i = 0; i < 10; i++) {
		disp_cnt_g[i] = 0;
	}

}

void ParticleFilter::initialize_filter_g() {

	if (init_g == 0) {
		x3_g = z3_g;
		x2_g = z2_g;
		init_g = 1;
	}
}

//void ParticleFilter::excution(float* var_lw){
//
//	int y_set;
//	int x_set;
//	float x_temp;
//	float x_model;
//	float sum_w = 0;
//
//	//printf("   > PF - State Transition \n");
//	int n_pp = 0;
//	for(int i=0;i<N_P;i++){
//		// State Transition
//		y_set = (int)((IMG_height-ROI_height)*i/N_P+ROI_height);
//		x_set = (int)(x[0]+x[1]*y_set+x[2]*y_set*y_set);
//		x_Update[n_pp][1] = y_set;
//		x_temp = x_set + var_lw[IMG_height-y_set]*randn();
//		x_Update[n_pp][0] = (int)x_temp;
//		
//		if( (x_set>IMG_width) || (x_set<0) ){
//			//printf(" > n_pp = %d, x_set = %d\n", n_pp, x_set);
//			break;
//		}
//		// Displaying - Transition
//		//cvCircle(image, cvPoint(x_Update[i][0],x_Update[i][1]), 1, CV_RGB(0,250,250), 4, 8, 0);
//
//		// Weighting
//		if( y_set > IMG_height*0.1+ROI_height*0.9 ){
//			x_model = z2[0]+z2[1]*y_set;
//		}else{
//			x_model = z[0]+z[1]*y_set+z[2]*y_set*y_set;
//		}
//		
//		if( abs(x_temp-x_model) <= 0.5 ){
//			x_P_W[n_pp] = 4*2*var_lw[IMG_height-y_set]*var_lw[IMG_height-y_set];	
//		}else{
//			x_P_W[n_pp] = 1/(x_temp-x_model)/(x_temp-x_model)*(2*var_lw[IMG_height-y_set]*var_lw[IMG_height-y_set]);	
//		}
//		sum_w = sum_w + x_P_W[n_pp];
//		n_pp++;
//	}
//	
//	//printf("   > PF - Weight Norm %d %f\n", n_pp,sum_w);
//	if( (sum_w < 0.01) || (n_pp < 10) ){
//		//printf("   > Weight sum is too small or too few particles\n" );
//		return;
//	}
//
//
//
//	// Weight Normalization
//	for(int i=0;i<n_pp;i++){
//		x_P_W[i] = x_P_W[i]/sum_w;
//		x_P_W_temp[i] = x_P_W[i];
//	}
//
//	//printf("   > Quicksort\n");
//	quicksort(x_P_W_temp,0,n_pp-1);
//
//	float thres_w = x_P_W_temp[(int)(n_pp*0.4)];
//	//printf(" >> %.3f \n", thres_w*N_P);
//	int n_p_temp=0;
//	for(int i=0;i<n_pp;i++){
//		if( x_P_W[i]> thres_w ){
//			x_P[n_p_temp][0] = x_Update[i][0];
//			x_P[n_p_temp][1] = x_Update[i][1];
//			n_p_temp++;
//			//cvCircle(image, cvPoint(x_Update[i][0],x_Update[i][1]), 1, CV_RGB(0,0,250), 4, 8, 0);
//		}else{
//			//cvCircle(image, cvPoint(x_Update[i][0],x_Update[i][1]), 1, CV_RGB(0,250,250), 4, 8, 0);
//		}
//	}
//
//	
//	//printf("   > Resulting Out %d\n", n_p_temp);
//	// Resulting Out
//	double* coeff3 = (double*)malloc(sizeof(double)*3);
//	double* coeff2 = (double*)malloc(sizeof(double)*3);
//	float** points = (float**)malloc(sizeof(float)*n_p_temp);
//	for(int i=0;i<n_p_temp;i++){
//		points[i] = (float*)malloc(sizeof(float)*2);
//	}
//	
//	// Calculation
//	for(int i=0;i<n_p_temp;i++){
//		points[i][0] = (float)x_P[i][0];
//		points[i][1] = (float)x_P[i][1];		
//	}
//	poly3(points,n_p_temp,coeff3);
//	poly2(points,n_p_temp,coeff2);
//
//	// Update
//	x[0] = (float)coeff3[0];
//	x[1] = (float)coeff3[1];
//	x[2] = (float)coeff3[2];
//	x2[0] = (float)coeff2[0];
//	x2[1] = (float)coeff2[1];
//	
//	// Releasing
//	for(int i=0;i<n_p_temp;i++){
//		free(points[i]);
//	}
//	free(points);
//	free(coeff3);
//	free(coeff2);
//}

void ParticleFilter::excution(std::vector<float>& var_lw, POLYNOMIAL& z3_, POLYNOMIAL& z2_, POLYNOMIAL& x3_, POLYNOMIAL& x2_) {
	
	// z_, z2_, x_, x2_ 는 *_g, *_y, *_b 중의 하나임
	std::vector<float> v_set(n_particle_);
	std::vector<float> u_set_x3(n_particle_);
	std::vector<float> u_set_z3(n_particle_); 
	std::vector<float> u_set_z2(n_particle_);	

	for (int ii = 0; ii < n_particle_; ++ii) {
		v_set[ii] = (height_ - height_roi_)*ii / n_particle_ + height_roi_;
	}
	fittingLinesWithPoly(v_set, u_set_x3, x3_);
	fittingLinesWithPoly(v_set, u_set_z3, z3_);
	fittingLinesWithPoly(v_set, u_set_z2, z2_);
	
	float sum_w = 0;

	//printf("   > PF - State Transition \n");
	int n_pp = 0;
	for (int ii = 0; ii < n_particle_; ++ii) {
		
		float val_lw_curr = var_lw[v_set[ii]];

		// State Transition		
		x_Update[n_pp].y = v_set[ii];
		float x_temp = u_set_x3[ii] + val_lw_curr * randn();
		x_Update[n_pp].x = (int)x_temp;

		if ((u_set_x3[ii] > width_ - 1) || (u_set_x3[ii] < 0)) {
			//printf(" > n_pp = %d, x_set = %d\n", n_pp, x_set);
			break;
		}

		// Weighting
		float x_model = 0.f;
		if (v_set[ii] > (height_)*0.3 + (height_roi_)*0.7) {
			x_model = u_set_z2[ii];
		}
		else {
			x_model = u_set_z3[ii];
		}

		if (abs(x_temp - x_model) <= 0.5) {
			x_P_W[n_pp] = 4 * 2 * val_lw_curr * val_lw_curr;
		}
		else {
			x_P_W[n_pp] = 1 / (x_temp - x_model) / (x_temp - x_model)*(2 * val_lw_curr * val_lw_curr);
		}
		sum_w = sum_w + x_P_W[n_pp];
		n_pp++;
	}

	//printf("   > PF - Weight Norm %d %f\n", n_pp,sum_w);
	if ((sum_w < 0.01) || (n_pp < 10)) {
		std::cout << sum_w << " " << n_pp << std::endl;
		printf("   > Weight sum is too small or too few particles\n" );
		return;
	}


	// Weight Normalization
	for (int i = 0; i < n_pp; i++) {
		x_P_W[i] = x_P_W[i] / sum_w;
		x_P_W_temp[i] = x_P_W[i];
	}

	// ToDo : sorting in the ascending order
	std::sort(x_P_W_temp.begin(), x_P_W_temp.end());
	

	float thres_w = x_P_W_temp[(int)(n_pp*0.7)];
	//printf(" >> %.3f \n", thres_w*N_P);
	int n_p_temp = 0;
	for (int i = 0; i < n_pp; i++) {
		if (x_P_W[i] > thres_w) {
			x_P[n_p_temp].x = x_Update[i].x;
			x_P[n_p_temp].y = x_Update[i].y;
			n_p_temp++;
			//cvCircle(image, cvPoint(x_Update[i][0],x_Update[i][1]), 1, CV_RGB(0,0,250), 4, 8, 0);
		}
		else {
			//cvCircle(image, cvPoint(x_Update[i][0],x_Update[i][1]), 1, CV_RGB(0,250,250), 4, 8, 0);
		}
	}
	
	//printf("   > Resulting Out %d\n", n_p_temp);
	// Resulting Out
	POLYNOMIAL poly3;
	POLYNOMIAL poly2;
	polynomial_initialize(poly3, 3);
	polynomial_initialize(poly2, 2);

	std::vector<cv::Point2f> pts(n_p_temp);
	
	// Calculation
	for (int ii = 0; ii < n_p_temp; ++ii) {
		pts[ii] = x_P[ii];
	}
	getPoly3_norm(pts, n_p_temp, poly3);
	getPoly2_norm(pts, n_p_temp, poly2);
	
	// Update
	x3_ = poly3;
	x2_ = poly2;

}

void ParticleFilter::final_validity() {

	int sum_g = 0;	
	for(int i=0;i<9;i++){
		disp_cnt_g[i] = disp_cnt_g[i+1]; 
		sum_g = sum_g + disp_cnt_g[i];
	}
	disp_cnt_g[9] = valid_g;
	sum_g = sum_g + disp_cnt_g[9];
	
	// Displaying Flag
	if( sum_g > 3 ){
		disp_g = 1;
	}else{
		init_g = 0;
		disp_g = 0;
		lane_type = 0;
	}	

	//// ROI Update
	//x2[0] = x2_g[0];
	//x2[1] = x2_g[1];	
}

float ParticleFilter::randn() {

	double v1, v2, s;

	do {
		v1 = 2 * ((double)rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
		v2 = 2 * ((double)rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
		s = v1 * v1 + v2 * v2;
	} while (s >= 1 || s == 0);

	s = sqrt((-2 * log(s)) / s);

	return (float)(v1 * s);
}
void ParticleFilter::quicksort(float temp_array[], int l, int r) {

	int left, right;
	float temp = 0;
	float pivot;
	if (l >= r) return;

	pivot = temp_array[l];
	left = l + 1;
	right = r;

	do {
		while (temp_array[left] <= pivot) {
			left++;
			if (left == r) {
				break;
			}
		}
		while (temp_array[right] >= pivot) {
			right--;
			if (right == l) {
				break;
			}
		}

		if (left < right) {
			temp = temp_array[left];
			temp_array[left] = temp_array[right];
			temp_array[right] = temp;
		}

	} while (left < right);

	temp = temp_array[right];
	temp_array[right] = temp_array[l];
	temp_array[l] = temp;
	quicksort(temp_array, l, right - 1);
	quicksort(temp_array, right + 1, r);
}

