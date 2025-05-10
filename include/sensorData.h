#pragma once

// c++ include
#include <iostream>
#include <vector>

// lib include
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

struct cameraData
{
	double timestamp;

	std::vector<int> camera_ids;

	std::vector<cv::Mat> images;

	std::vector<cv::Mat> masks;

	bool operator<(const cameraData &other) const {
		if (timestamp == other.timestamp) {
			int id = *std::min_element(camera_ids.begin(), camera_ids.end());
			int id_other = *std::min_element(other.camera_ids.begin(), other.camera_ids.end());
			return id < id_other;
		} else {
			return timestamp < other.timestamp;
		}
	}
};

struct imuData
{
	double timestamp;

	Eigen::Matrix<double, 3, 1> gyr;

	Eigen::Matrix<double, 3, 1> acc;

	bool operator<(const imuData &other) const {
		return timestamp < other.timestamp;
	}
};

static imuData interpolateData(const imuData &imu_1, const imuData &imu_2, double timestamp);

std::vector<imuData> selectImuReadings(const std::vector<imuData> &v_imu_data_, double time_0, double time_1, bool warn);