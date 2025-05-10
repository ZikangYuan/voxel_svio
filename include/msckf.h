#pragma once

// c++ include
#include <iostream>
#include <cmath>
#include <atomic>

// lib include
#include <boost/math/distributions/chi_squared.hpp>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// function include
#include "state.h"
#include "feature.h"
#include "featureHelper.h"
#include "mapPoint.h"
#include "utility.h"
#include "quatOps.h"
#include "sensorData.h"
#include "parameters.h"
#include "updaterHelper.h"

class propagator
{
public:

  propagator(noiseManager noises_, double gravity_mag_);

  void feedImu(const imuData &imu_data, double oldest_time = -1);

  void cleanOldImuMeasurements(double oldest_time);

  void invalidateCache();

  void resetTransitionMatrices(std::shared_ptr<state> state_ptr);

  static imuData interpolateData(const imuData &imu_1, const imuData &imu_2, double timestamp);

  static std::vector<imuData> selectImuReadings(const std::vector<imuData> &v_imu_data_, double time_0, double time_1, bool warn = true);

  void propagateAndClone(std::shared_ptr<state> state_ptr, double timestamp);

  void predictAndCompute(std::shared_ptr<state> state_ptr, const imuData &data_minus, const imuData &data_plus, 
    Eigen::MatrixXd &F, Eigen::MatrixXd &Qd);

  static Eigen::MatrixXd computeDwH(std::shared_ptr<state> state_ptr, const Eigen::Vector3d &gyr_uncorrected);

  static Eigen::MatrixXd computeDaH(std::shared_ptr<state> state_ptr, const Eigen::Vector3d &acc_uncorrected);

  static Eigen::MatrixXd computeTgH(std::shared_ptr<state> state_ptr, const Eigen::Vector3d &acc);

  bool have_last_prop_time_offset;
  double last_prop_time_offset;

protected:

  void predictMeanRk4(std::shared_ptr<state> state_ptr, double dt, const Eigen::Vector3d &un_gyr_0, const Eigen::Vector3d &un_acc_0, 
    const Eigen::Vector3d &un_gyr_1, const Eigen::Vector3d &un_acc_1, Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p);

  void computeXiSum(std::shared_ptr<state> state_ptr, double dt, const Eigen::Vector3d &un_gyr, 
    const Eigen::Vector3d &un_acc, Eigen::Matrix<double, 3, 18> &Xi_sum);

  void computeCoefficientAnalytic(std::shared_ptr<state> state_ptr, double dt, const Eigen::Vector3d &un_gyr, const Eigen::Vector3d &un_acc, 
    const Eigen::Vector3d &gyr_uncorrected, const Eigen::Vector3d &acc_uncorrected, const Eigen::Vector4d &new_q, const Eigen::Vector3d &new_v, 
    const Eigen::Vector3d &new_p, const Eigen::Matrix<double, 3, 18> &Xi_sum, Eigen::MatrixXd &F, Eigen::MatrixXd &G);

  noiseManager noises;

  std::vector<imuData> v_imu_data;

  Eigen::Vector3d gravity;

  std::atomic<bool> cache_imu_valid;
  double cache_state_time;
  Eigen::MatrixXd cache_state_est;
  Eigen::MatrixXd cache_state_covariance;
  double cache_t_off;
};

class updaterMsckf
{
public:

  updaterMsckf(updaterOptions &options_, featureInitializerOptions &feature_init_options_);

  void update(std::shared_ptr<state> state_ptr, std::vector<std::shared_ptr<feature>> &feature_vec);

protected:

  updaterOptions options;

  std::shared_ptr<featureInitializer> initializer_feature;

  std::map<int, double> chi_squared_table;
};

class updaterSlam
{
public:

  updaterSlam(updaterOptions &options_slam_, featureInitializerOptions &feat_init_options_, std::shared_ptr<featureDatabase> database_);

  void update(std::shared_ptr<state> state_ptr, voxelHashMap &voxel_map, std::vector<std::shared_ptr<feature>> &feature_vec);

  void delayedInit(std::shared_ptr<state> state_ptr, voxelHashMap &voxel_map, std::vector<std::shared_ptr<feature>> &feature_vec, pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history);

protected:

  updaterOptions options_slam;

  std::shared_ptr<featureInitializer> initializer_feature;

  std::shared_ptr<featureDatabase> database;

  std::map<int, double> chi_squared_table;
};