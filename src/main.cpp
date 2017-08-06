#pragma warning(disable:4819)
#pragma once
#include <string>
#include <opencv2\opencv.hpp>
#include <iostream>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include "LaneDetection.h"

#include <json/json.h>
#include <fstream>


void run_ld(std::string data_path, Json::Value& currQuery, std::ofstream& output_file){

	// input parameters	
	std::string final_frame = data_path + currQuery["raw_file"].asString();
	std::string image_path = final_frame.substr(0, final_frame.size() - 6);

	bool mode_color = false;
	float height_roi_ratio = 0.37f;
	//float height_roi_ratio = 0.40f;

	// reading image resolution info
	cv::Mat testImg = cv::imread(image_path + "1.jpg");
	if (testImg.empty()) {
		std::cout << " Fatal Error: no Input image: " << image_path + "1.jpg" << std::endl;
		return;
	}
	cv::Size size_img = testImg.size();

	clock_t start, finish, processing_time = 0;
	int frame_num = 0;

	// LD Variables definition and initialization
	LaneDetection* ld = new LaneDetection();
	ld->initialize_variable(testImg, mode_color, height_roi_ratio);

	int loop_select = 1;
	int loop1_init = 1;	// 1: should be initialized, 0: initailized
	int loop2_init = 1;	// 1: should be initialized, 0: initailized
	

	// run
	for (int n = 1; n<21; n++){

		//Image Sourcing
		std::string img_name = image_path + std::to_string(n) + ".jpg";
		cv::Mat img_src = cv::imread(img_name);
		if (img_src.empty()) {
			std::cout << " Fatal Error: no Input image: " << img_src << std::endl;
			break;
		}
		start = clock();

		ld->initialize_img(img_src);
		// LOOP 1 -- driving lane full search
		if (loop_select == 1){
			loop_select = ld->FS_driving_lane_detection(loop1_init);
			loop1_init = 0;
		}

		// LOOP 2
		if (loop_select == 2){
			loop_select = ld->HG_HV_lane_detection(img_src, loop2_init);
			loop2_init = 0;

			if (loop_select == 1){
				loop1_init = 1;
			}

			// displaying
			ld->displaying();
		}

		finish = clock();
		processing_time = finish - start;
		//std::cout << " processing time: " << (float)processing_time / (float)(CLOCKS_PER_SEC / 1000.f) << std::endl;
		cv::waitKey(1);
	}

	ld->writingToOutputFile(currQuery, output_file, (float)processing_time / (float)(CLOCKS_PER_SEC / 1000.f));


	// Common Variables 
	ld->~LaneDetection();

}


void main(){ 

	bool bWrite = false;

	// directory setting
	std::string data_path = "../data/train_set/";
	std::string data_file_name = data_path + "train_task.json";
	//std::string data_path = "U:/programming/data/tuSimple/test_set/";
	//std::string data_file_name = data_path + "test_tasks_0627.json";
	std::string output_file_name = data_path + "output.json";

	// tmp
	std::string output_file_name_tmp1 = data_path + "output_temp1.json";
	std::string output_file_name_tmp2 = data_path + "output_temp2.json";

	// Json File read
	std::vector<Json::Value> data_input_jason;
	Json::Value currVal;
	Json::Reader reader;
	std::ifstream filename_jason(data_file_name, std::ifstream::binary);
	std::ofstream output_file(output_file_name);
	

	if (!filename_jason.is_open()){
		std::cout << " cannot open the file " << data_file_name << std::endl;
	}
	std::string cur_line;
	bool success;

	do {
		std::getline(filename_jason, cur_line);
		success = reader.parse(cur_line, currVal, false);
		data_input_jason.push_back(currVal);
	} while (success);
	
	std::cout << data_input_jason.size() << std::endl;

	for (int ii = 0; ii < data_input_jason.size(); ++ii){

		std::ofstream output_file_tmp(output_file_name_tmp1);
		run_ld(data_path, data_input_jason[ii], output_file_tmp);
		output_file_tmp.close();
		cv::waitKey(0);

		// saving
		if (bWrite) {
			std::ifstream filename_tmp1(output_file_name_tmp1);
			std::string cur_line;
			while (std::getline(filename_tmp1, cur_line)) {
				output_file << cur_line;
			}
			output_file << std::endl;
			filename_tmp1.close();
		}
	}

	output_file.close();

}