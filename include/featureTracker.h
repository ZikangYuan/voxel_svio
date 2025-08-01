#pragma once

// c++ include
#include <iostream>
#include <atomic>
#include <thread>
#include <unordered_map>

// lib include
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// function include
#include "utility.h"
#include "sensorData.h"
#include "frame.h"
#include "pixelSelector.h"

class feature;
class featureDatabase;
class cameraBase;
class frame;

class trackKLT
{
public:

	trackKLT(std::unordered_map<size_t, std::shared_ptr<cameraBase>> camera_calib_, int num_features_, 
		HistogramMethod histogram_method_, int fast_threshold_, int patch_size_x_, int patch_size_y_, int min_px_dist_);

	void feedNewImage(const cameraData &image_measurements, std::shared_ptr<frame> fh);

	std::shared_ptr<featureDatabase> getFeatureDatabase();

	std::unordered_map<size_t, std::vector<cv::KeyPoint>> getLastObs();

	std::unordered_map<size_t, std::vector<size_t>> getLastIds();

	void setNumFeatures(int num_features_);

	void displayActive(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay = "");

private:
	void feedStereo(const cameraData &image_measurements, std::shared_ptr<frame> fh, size_t image_id_left, size_t image_id_right);

	void performDetectionStereo(std::shared_ptr<frame> fh, const std::vector<cv::Mat> &img_0_pyr, const std::vector<cv::Mat> &img_1_pyr, const cv::Mat &mask_0, const cv::Mat &mask_1, 
		size_t cam_id_left, size_t cam_id_right, std::vector<cv::KeyPoint> &pts_0, std::vector<cv::KeyPoint> &pts_1, std::vector<size_t> &ids_0, std::vector<size_t> &ids_1);

	void performMatching(const std::vector<cv::Mat> &img_0_pyr, const std::vector<cv::Mat> &img_1_pyr, std::vector<cv::KeyPoint> &pts_0, 
		std::vector<cv::KeyPoint> &pts_1, size_t id_0, size_t id_1, std::vector<uchar> &mask_out);

	std::unordered_map<size_t, std::shared_ptr<cameraBase>> camera_calib;

	std::shared_ptr<featureDatabase> database;

	int num_features;

	HistogramMethod histogram_method;

	std::map<size_t, cv::Mat> img_last;

	std::map<size_t, cv::Mat> img_mask_last;

	std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last;

	std::unordered_map<size_t, std::vector<size_t>> ids_last;

	std::atomic<size_t> current_id;

	std::shared_ptr<pixelSelector> pixelSelector_ptr;

	int threshold;
	int patch_size_x;
	int patch_size_y;

	int min_px_dist;

	int pyr_levels = 5;
	cv::Size win_size = cv::Size(15, 15);

	std::map<size_t, std::vector<cv::Mat>> img_pyramid_last;
	std::map<size_t, cv::Mat> img_curr;
	std::map<size_t, std::vector<cv::Mat>> img_pyramid_curr;

	std::shared_ptr<frame> fh_last;

	boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;
};
