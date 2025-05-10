#pragma once

// c++ include
#include <iostream>
#include <unordered_map>

// lib include
#include <boost/filesystem.hpp>

// function include
#include "utility.h"
#include "camera.h"

class stateOptions
{
public:
    
    bool do_fej = true;

    bool do_calib_camera_pose = false;

    bool do_calib_camera_intrinsics = false;

    bool do_calib_camera_timeoffset = false;

    bool do_calib_imu_intrinsics = false;

    bool do_calib_imu_g_sensitivity = false;

    ImuModel imu_model = ImuModel::KALIBR;

    int max_clone_size = 11;

    int max_slam_features = 25;

    int max_slam_in_update = 1000;

    int max_msckf_in_update = 1000;

    double voxel_size = 0.5;

    int max_num_points_in_voxel = 10;

    double min_distance_points = 0.15;

    bool use_huber = true;

    void recordParameters();
};

class inertialInitializerOptions
{
public:

    double init_window_time = 1.0;

    double init_imu_thresh = 1.0;

    double init_max_disparity = 1.0;

    int init_max_features = 50;

    bool init_dyn_use = false;

    bool init_dyn_mle_opt_calib = false;

    int init_dyn_mle_max_iter = 20;

    int init_dyn_mle_max_threads = 20;

    double init_dyn_mle_max_time = 5.0;

    int init_dyn_num_pose = 5;

    double init_dyn_min_deg = 45.0;

    double init_dyn_inflation_orientation = 10.0;

    double init_dyn_inflation_velocity = 10.0;

    double init_dyn_inflation_bias_gyro = 100.0;

    double init_dyn_inflation_bias_accel = 100.0;

    double init_dyn_min_rec_cond = 1e-15;

    Eigen::Vector3d init_dyn_bias_g = Eigen::Vector3d::Zero();

    Eigen::Vector3d init_dyn_bias_a = Eigen::Vector3d::Zero();

    double sigma_w = 1.6968e-04;

    double sigma_wb = 1.9393e-05;

    double sigma_a = 2.0000e-3;

    double sigma_ab = 3.0000e-03;

    double sigma_pix = 1;

    double gravity_mag = 9.81;

    bool downsample_cameras = false;

    double calib_camimu_dt = 0.0;

    std::unordered_map<size_t, std::shared_ptr<cameraBase>> camera_intrinsics;

    std::map<size_t, Eigen::VectorXd> camera_extrinsics;

    void recordParameters();
};

class featureInitializerOptions
{
public:

    bool refine_features = true;

    int max_runs = 5;

    double init_lamda = 1e-3;

    double max_lamda = 1e10;

    double min_dx = 1e-6;

    double min_dcost = 1e-6;

    double lam_mult = 10;

    double min_dist = 0.10;

    double max_dist = 60;

    double max_baseline = 40;

    double max_cond_number = 10000;

    void recordParameters();
};

class noiseManager
{
public:

    double sigma_w = 1.6968e-04;

    double sigma_w_2 = pow(1.6968e-04, 2);

    double sigma_wb = 1.9393e-05;

    double sigma_wb_2 = pow(1.9393e-05, 2);

    double sigma_a = 2.0000e-3;

    double sigma_a_2 = pow(2.0000e-3, 2);

    double sigma_ab = 3.0000e-03;

    double sigma_ab_2 = pow(3.0000e-03, 2);

    void recordParameters();
};

class updaterOptions
{
public:

    double chi2_multipler = 5;

    double sigma_pix = 1;

    double sigma_pix_sq = 1;

    void recordParameters();
};

class odometryOptions
{
public:

    stateOptions state_options;

    inertialInitializerOptions init_options;

    double dt_slam_delay = 2.0;

    noiseManager imu_noises;

    updaterOptions msckf_options;

    updaterOptions slam_options;

    double gravity_mag = 9.81;

    Eigen::Matrix<double, 6, 1> vec_dw;

    Eigen::Matrix<double, 6, 1> vec_da;

    Eigen::Matrix<double, 9, 1> vec_tg;

    Eigen::Matrix<double, 4, 1> q_imu_acc;

    Eigen::Matrix<double, 4, 1> q_imu_gyr;

    double calib_camimu_dt = 0.0;

    std::unordered_map<size_t, std::shared_ptr<cameraBase>> camera_intrinsics;

    std::map<size_t, Eigen::VectorXd> camera_extrinsics;

    bool use_mask = false;

    std::map<size_t, cv::Mat> masks;

    bool downsample_cameras = false;

    int num_pts = 150;

    int fast_threshold = 20;

    int patch_size_x = 5;

    int patch_size_y = 5;

    int min_px_dist = 10;

    HistogramMethod histogram_method = HistogramMethod::HISTOGRAM;

    double track_frequency = 20.0;

    featureInitializerOptions featinit_options;

    double voxel_size = 0.5;

    int max_num_points_in_voxel = 10;

    double min_distance_points = 0.15;

    int nb_voxels_visited = 1;

    bool use_all_points = false;

    bool use_keyframe = true;

    void recordParameters();
};