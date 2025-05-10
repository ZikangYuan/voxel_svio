#pragma once

// c++ include
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// lib include
#include <Eigen/Core>

// function include
#include "state.h"
#include "sensorData.h"

class initializerHelper
{
public:

  static imuData interpolateData(const imuData &imu_1, const imuData &imu_2, double timestamp); // interpolate_data

  static std::vector<imuData> selectImuReadings(const std::vector<imuData> &imu_data_tmp, double time_0, double time_1);

  static void gramSchmidt(const Eigen::Vector3d &gravity_inI, Eigen::Matrix3d &R_GtoI);

  static Eigen::Matrix<double, 7, 1> computeDongsiCoeff(Eigen::MatrixXd &D, const Eigen::MatrixXd &d, double gravity_mag);
};