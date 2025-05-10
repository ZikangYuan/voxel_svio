#pragma once

// c++ include
#include <iostream>
#include <unordered_map>
#include <sstream>
#include <string>
#include <vector>

// lib include
#include <Eigen/Core>

// function include
#include "utility.h"
#include "parameters.h"
#include "quatOps.h"
#include "frame.h"

class state;

class feature
{
public:

  size_t feature_id;

  bool to_delete;

  std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs;

  std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs_norm;

  std::unordered_map<size_t, std::vector<double>> timestamps;

  std::unordered_map<size_t, std::vector<std::shared_ptr<frame>>> frames;

  std::unordered_map<size_t, std::vector<float>> color;

  int start_frame_id;

  int start_cam_id;

  int num_tracking;

  int anchor_cam_id = -1;

  double anchor_clone_timestamp;

  Eigen::Vector3d position_anchor;

  Eigen::Vector3d position_global;

  void cleanOldMeasurements(const std::vector<double> &valid_times);

  void cleanInvalidMeasurements(const std::vector<double> &invalid_times);

  void cleanOlderMeasurements(double timestamp);
};

class featureDatabase
{
public:

  featureDatabase();

  std::shared_ptr<feature> getFeature(size_t id, bool remove = false);

  void updateFeature(std::shared_ptr<frame> fh, size_t id, double timestamp, size_t cam_id, float u, float v, float u_n, float v_n);

  std::vector<std::shared_ptr<feature>> getOldFeatures(double timestamp, bool remove = false, bool skip_deleted = false);

  std::vector<std::shared_ptr<feature>> getFeatures(double timestamp, bool remove = false, bool skip_deleted = false);

  void cleanUp();

  void cleanUpOldmeasurements(double timestamp);

  size_t getSize();

  std::unordered_map<size_t, std::shared_ptr<feature>> getInternalData();

  bool featureCheckParallax(int frame_count);

  double compensatedParallax2(int feat_id, int cam_id, int frame_count);

protected:

  std::unordered_map<size_t, std::shared_ptr<feature>> features_id_lookup;

  int last_track_num;
};

class featureInitializer
{
public:

  struct clonePose
  {
    Eigen::Matrix<double, 3, 3> rotation;

    Eigen::Matrix<double, 3, 1> position;

    clonePose(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &p)
    {
      rotation = R;
      position = p;
    }

    clonePose(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 3, 1> &p)
    {
      rotation = quatType::quatToRot(q);
      position = p;
    }

    clonePose()
    {
      rotation = Eigen::Matrix<double, 3, 3>::Identity();
      position = Eigen::Matrix<double, 3, 1>::Zero();
    }

    const Eigen::Matrix<double, 3, 3> &Rot() { return rotation; }

    const Eigen::Matrix<double, 3, 1> &pos() { return position; }
  };

  featureInitializer(featureInitializerOptions &options_);

  bool singleTriangulation(std::shared_ptr<feature> feat, std::unordered_map<size_t, std::unordered_map<double, clonePose>> &clones_cam);

  bool singleGaussnewton(std::shared_ptr<feature> feat, std::unordered_map<size_t, std::unordered_map<double, clonePose>> &clones_cam);

  const featureInitializerOptions config() { return options; }

protected:

  featureInitializerOptions options;

  double computeError(std::unordered_map<size_t, std::unordered_map<double, clonePose>> &clones_cam, std::shared_ptr<feature> feat,
    double alpha, double beta, double rho);
};