#include "utility.h"

std::string output_path;

bool initial_flag = false;

int pyr_levels_used = 1;

int wG[PYR_LEVELS], hG[PYR_LEVELS];

bool setting_gamma_pixel_selection = true;

float setting_min_grad_hist_cut = 0.5;

float setting_min_grad_hist_add = 7;

float setting_grad_down_weight_per_level = 0.75;

bool  setting_select_direction_distribution = true;

float setting_huber_th = 1;

float setting_min_parallax = 10;

gammaPixel::gammaPixel()
{
    for (int i = 0; i < 256; i++)
        Binv[i] = B[i] = i;
}

float gammaPixel::getBGradOnly(float color)
{
    int c = color + 0.5f;
    if(c < 5) c = 5;
    if(c > 250) c = 250;
    return B[c + 1] - B[c];
}

float gammaPixel::getBInvGradOnly(float color)
{
    int c = color + 0.5f;
    if(c < 5) c = 5;
    if(c > 250) c = 250;
    return Binv[c + 1] - Binv[c];
}

cv::Mat getCvImage(float* temp_image)
{
    cv::Mat img_gray(hG[0], wG[0], CV_32FC1, temp_image);
    img_gray.convertTo(img_gray, CV_8UC1);

    cv::Mat img_color(hG[0], wG[0], CV_8UC3);
    for (int i = 0; i < hG[0]; i++)
    {
        for (int j = 0; j < wG[0]; j++)
            img_color.at<cv::Vec3b>(i, j) = cv::Vec3b(img_gray.at<uchar>(i, j), img_gray.at<uchar>(i, j), img_gray.at<uchar>(i, j));
    }

    img_gray.release();

    return img_color;
}

cv::Mat convertCvImage(cv::Mat &img_gray)
{
    cv::Mat img_color(hG[0], wG[0], CV_8UC3);
    for (int i = 0; i < hG[0]; i++)
    {
        for (int j = 0; j < wG[0]; j++)
            img_color.at<cv::Vec3b>(i, j) = cv::Vec3b(img_gray.at<uchar>(i, j), img_gray.at<uchar>(i, j), img_gray.at<uchar>(i, j));
    }

    return img_color;
}

void drawPoint(cv::Mat &img, int u, int v, cv::Vec3b &color)
{
    img.at<cv::Vec3b>(v, u) = color;

    img.at<cv::Vec3b>(v + 2, u + 2) = color;
    img.at<cv::Vec3b>(v + 2, u + 1) = color;
    img.at<cv::Vec3b>(v + 2, u) = color;
    img.at<cv::Vec3b>(v + 2, u - 1) = color;
    img.at<cv::Vec3b>(v + 2, u - 2) = color;

    img.at<cv::Vec3b>(v + 1, u + 2) = color;
    img.at<cv::Vec3b>(v + 1, u - 2) = color;

    img.at<cv::Vec3b>(v, u + 2) = color;
    img.at<cv::Vec3b>(v, u - 2) = color;

    img.at<cv::Vec3b>(v - 1, u + 2) = color;
    img.at<cv::Vec3b>(v - 1, u - 2) = color;

    img.at<cv::Vec3b>(v - 2, u + 2) = color;
    img.at<cv::Vec3b>(v - 2, u + 1) = color;
    img.at<cv::Vec3b>(v - 2, u) = color;
    img.at<cv::Vec3b>(v - 2, u - 1) = color;
    img.at<cv::Vec3b>(v - 2, u - 2) = color;
}

void drawPointL(cv::Mat &img, int u, int v, cv::Vec3b &color)
{
    img.at<cv::Vec3b>(v, u) = color;

    img.at<cv::Vec3b>(v + 3, u + 3) = color;
    img.at<cv::Vec3b>(v + 3, u + 2) = color;
    img.at<cv::Vec3b>(v + 3, u + 1) = color;
    img.at<cv::Vec3b>(v + 3, u) = color;
    img.at<cv::Vec3b>(v + 3, u - 1) = color;
    img.at<cv::Vec3b>(v + 3, u - 2) = color;
    img.at<cv::Vec3b>(v + 3, u - 3) = color;

    img.at<cv::Vec3b>(v + 2, u + 3) = color;
    img.at<cv::Vec3b>(v + 2, u - 3) = color;

    img.at<cv::Vec3b>(v + 1, u + 3) = color;
    img.at<cv::Vec3b>(v + 1, u - 3) = color;

    img.at<cv::Vec3b>(v, u + 3) = color;
    img.at<cv::Vec3b>(v, u - 3) = color;

    img.at<cv::Vec3b>(v - 1, u + 3) = color;
    img.at<cv::Vec3b>(v - 1, u - 3) = color;

    img.at<cv::Vec3b>(v - 2, u + 3) = color;
    img.at<cv::Vec3b>(v - 2, u - 3) = color;

    img.at<cv::Vec3b>(v - 3, u + 3) = color;
    img.at<cv::Vec3b>(v - 3, u + 2) = color;
    img.at<cv::Vec3b>(v - 3, u + 1) = color;
    img.at<cv::Vec3b>(v - 3, u) = color;
    img.at<cv::Vec3b>(v - 3, u - 1) = color;
    img.at<cv::Vec3b>(v - 3, u - 2) = color;
    img.at<cv::Vec3b>(v - 3, u - 3) = color;
}

Eigen::Matrix4d mat44FromArray(std::vector<double> &array)
{
    assert(array.size() == 16);
    Eigen::Matrix4d mat44;
    mat44(0, 0) = array[0]; mat44(0, 1) = array[1]; mat44(0, 2) = array[2]; mat44(0, 3) = array[3];
    mat44(1, 0) = array[4]; mat44(1, 1) = array[5]; mat44(1, 2) = array[6]; mat44(1, 3) = array[7];
    mat44(2, 0) = array[8]; mat44(2, 1) = array[9]; mat44(2, 2) = array[10]; mat44(2, 3) = array[11];
    mat44(3, 0) = array[12]; mat44(3, 1) = array[13]; mat44(3, 2) = array[14]; mat44(3, 3) = array[15];

    return mat44;
}

Eigen::Matrix3d mat33FromArray(std::vector<double> &array)
{
    assert(array.size() == 9);
    Eigen::Matrix3d mat33;
    mat33(0, 0) = array[0]; mat33(0, 1) = array[1]; mat33(0, 2) = array[2];
    mat33(1, 0) = array[3]; mat33(1, 1) = array[4]; mat33(1, 2) = array[5];
    mat33(2, 0) = array[6]; mat33(2, 1) = array[7]; mat33(2, 2) = array[8];

    return mat33;
}

Eigen::Vector3d vec3FromArray(std::vector<double> &array)
{
    assert(array.size() == 3);
    Eigen::Vector3d vec3;
    vec3(0, 0) = array[0]; vec3(1, 0) = array[1]; vec3(2, 0) = array[2];

    return vec3;
}