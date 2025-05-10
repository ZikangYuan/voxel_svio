#pragma once

// c++ include
#include <iostream>
#include <string>
#include <math.h>
#include <vector>

// lib include
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tr1/unordered_map>

// function include
#include "utility.h"
#include "frame.h"

class pixelSelector
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	pixelSelector(int w, int h);
	~pixelSelector();

	int pixelSelectionLeft(std::shared_ptr<frame> fh, float density, int recursions_left = 1, float th_factor = 1);

	int pixelSelectionRight(std::shared_ptr<frame> fh, float density, int recursions_left = 1, float th_factor = 1);

private:

	void makeHistsLeft(std::shared_ptr<frame> fh);

	void makeHistsRight(std::shared_ptr<frame> fh);

	Eigen::Vector3i selectLeft(std::shared_ptr<frame> fh, int pot, float th_factor = 1);

	Eigen::Vector3i selectRight(std::shared_ptr<frame> fh, int pot, float th_factor = 1);

	int current_potential;

	bool allow_fast;

	unsigned char* random_pattern;

	int* grad_hist;
	float* ths;
	float* ths_smoothed;
	int ths_step;
	std::shared_ptr<frame> grad_hist_frame;
};