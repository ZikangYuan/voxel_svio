#pragma once

// c++ include
#include <iostream>
#include <vector>

// lib include
#include <Eigen/Core>

// function include
#include "feature.h"

class featureHelper
{
public:

  static void computeDisparity(std::shared_ptr<featureDatabase> db, double time_0, double time_1, double &disp_mean, double &disp_var, int &total_feats);

  static void computeDisparity(std::shared_ptr<featureDatabase> db, double &disp_mean, double &disp_var, int &total_feats,
    double newest_time = -1, double oldest_time = -1);

private:

  featureHelper();
};