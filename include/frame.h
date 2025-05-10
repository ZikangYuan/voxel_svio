#pragma once

// c++ include
#include <iostream>

// lib include
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

// function include
#include "utility.h"

class feature;

class frame
{
public:

	Eigen::Vector3f* dI_left;
	Eigen::Vector3f* dI_right;

	Eigen::Vector3f* dIp_left[PYR_LEVELS];
	Eigen::Vector3f* dIp_right[PYR_LEVELS];

	float* abs_squared_grad_left[PYR_LEVELS];
	float* abs_squared_grad_right[PYR_LEVELS];

	bool* mask_left;
	bool* mask_right;

	float* selected_pixels_left;
	float* selected_pixels_right;

	int frame_id;

	double timestamp;

	bool is_invalid = false;

	std::unordered_map<size_t, std::unordered_map<size_t, std::shared_ptr<feature>>> v_feat_ptr;

	void release();

	void makeImageLeft(unsigned char* color, unsigned char* mask, std::shared_ptr<gammaPixel> gammaPixel_ptr, double timestamp_);

	void makeImageRight(unsigned char* color, unsigned char* mask, std::shared_ptr<gammaPixel> gammaPixel_ptr, double timestamp_);
};