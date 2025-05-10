#pragma once

// c++ include
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// lib include
#include <Eigen/Core>

// function include
#include "utility.h"
#include "parameters.h"
#include "sensorData.h"
#include "state.h"
#include "preIntegration.h"
#include "camera.h"
#include "featureTracker.h"
#include "mapPoint.h"
#include "utility.h"
#include "quatOps.h"

class staticInitializer;
class dynamicInitializer;

class inertialInitializer
{
public:

  inertialInitializer(inertialInitializerOptions &initializer_options_, std::shared_ptr<featureDatabase> db_);

  void feedImu(const imuData &imu_data_, double oldest_time = -1);

  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<baseType>> &order,
    std::shared_ptr<imuState> imu_state_, bool wait_for_jerk = true);

protected:

  inertialInitializerOptions initializer_options;

  std::shared_ptr<featureDatabase> db;

  std::shared_ptr<std::vector<imuData>> imu_data;

  std::shared_ptr<staticInitializer> static_initializer;

  std::shared_ptr<dynamicInitializer> dynamic_initializer;
};

class staticInitializer
{
public:

  explicit staticInitializer(inertialInitializerOptions &initializer_options_, std::shared_ptr<featureDatabase> db_,
    std::shared_ptr<std::vector<imuData>> imu_data_);

  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<baseType>> &order, 
    std::shared_ptr<imuState> imu_state_, bool wait_for_jerk = true);

private:

  inertialInitializerOptions initializer_options;

  std::shared_ptr<featureDatabase> db;

  std::shared_ptr<std::vector<imuData>> imu_data;
};

class dynamicInitializer
{
public:

  explicit dynamicInitializer(const inertialInitializerOptions &initializer_options_, std::shared_ptr<featureDatabase> db_, 
    std::shared_ptr<std::vector<imuData>> imu_data_);

  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<baseType>> &order, 
    std::shared_ptr<imuState> &imu_state_, std::map<double, std::shared_ptr<poseJpl>> &clones_imu_, 
    std::unordered_map<size_t, std::shared_ptr<mapPoint>> &map_points_);

private:

  inertialInitializerOptions initializer_options;

  std::shared_ptr<featureDatabase> db;

  std::shared_ptr<std::vector<imuData>> imu_data;
};