#pragma once

// c++ include
#include <iostream>
#include <unordered_map>

// lib include
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <ceres/ceres.h>

// function include
#include "utility.h"

class cameraBase
{
public:

	cameraBase(int width, int height);

	virtual ~cameraBase() {}

	virtual void setValue(const Eigen::MatrixXd &calib);

	virtual Eigen::Vector2f undistortF(const Eigen::Vector2f &uv_dist) = 0;

	Eigen::Vector2d undistortD(const Eigen::Vector2d &uv_dist);

	cv::Point2f undistortCV(const cv::Point2f &uv_dist);

	virtual Eigen::Vector2f distortF(const Eigen::Vector2f &uv_norm) = 0;

	Eigen::Vector2d distortD(const Eigen::Vector2d &uv_norm);

	cv::Point2f distortCV(const cv::Point2f &uv_norm);

	virtual void computeDistortJacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) = 0;

	Eigen::MatrixXd getValue();

	cv::Matx33d getK();

	cv::Vec4d getD();

	int w();

	int h();

protected:

	cameraBase() = default;

	Eigen::MatrixXd camera_values; // fx fy cx cy k1 k2 k3 k4

	cv::Matx33d camera_k_OPENCV;

	cv::Vec4d camera_d_OPENCV;

	int width;

	int height;
};

class cameraEqui : public cameraBase
{
public:

	cameraEqui(int width, int height) : cameraBase(width, height) {}

	~cameraEqui() {}

 	Eigen::Vector2f undistortF(const Eigen::Vector2f &uv_dist) override;

 	Eigen::Vector2f distortF(const Eigen::Vector2f &uv_norm) override;

 	void computeDistortJacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) override;
};

class cameraRadtan : public cameraBase
{
public:

	cameraRadtan(int width, int height) : cameraBase(width, height) {}

	~cameraRadtan() {}

	Eigen::Vector2f undistortF(const Eigen::Vector2f &uv_dist) override;

	Eigen::Vector2f distortF(const Eigen::Vector2f &uv_norm) override;

 	void computeDistortJacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) override;
};