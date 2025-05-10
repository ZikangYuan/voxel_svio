#pragma once

// c++ include
#include <iostream>
#include <string>
#include <tr1/unordered_map>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <functional>

// lib include
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#define PYR_LEVELS 6

enum ImuModel
{
    KALIBR,
    RPNG
};

enum HistogramMethod
{ 
    NONE,
    HISTOGRAM,
    CLAHE
};

enum PixelSelectorStatus {
    PIXSEL_VOID,
    PIXSEL_1,
    PIXSEL_2, 
    PIXSEL_3
};

enum MarginalizeStatus
{
    OLD,
    NEW
};

extern std::string output_path;

extern bool initial_flag;

extern int pyr_levels_used;

extern int wG[PYR_LEVELS], hG[PYR_LEVELS];

extern bool setting_gamma_pixel_selection;

extern float setting_min_grad_hist_cut;

extern float setting_min_grad_hist_add;

extern float setting_grad_down_weight_per_level;

extern bool  setting_select_direction_distribution;

extern float setting_huber_th;

extern float setting_min_parallax;

class gammaPixel
{
public:
    float Binv[256];
    float B[256];

    gammaPixel();

    float getBGradOnly(float color);

    float getBInvGradOnly(float color);
};

class lambdaBody : public cv::ParallelLoopBody
{
public:
    explicit lambdaBody(const std::function<void(const cv::Range &)> &body_) { body = body_; }
    void operator()(const cv::Range &range) const override { body(range); }

private:
    std::function<void(const cv::Range &)> body;
};

cv::Mat getCvImage(float* temp_image);

cv::Mat convertCvImage(cv::Mat &img_gray);

void drawPoint(cv::Mat &img, int u, int v, cv::Vec3b &color);

void drawPointL(cv::Mat &img, int u, int v, cv::Vec3b &color);

Eigen::Matrix4d mat44FromArray(std::vector<double> &array);

Eigen::Matrix3d mat33FromArray(std::vector<double> &array);

Eigen::Vector3d vec3FromArray(std::vector<double> &array);

namespace std {
    template <typename T, typename... Args>
        std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}