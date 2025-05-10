#pragma once

// c++ include
#include <iostream>
#include <cmath>
#include <memory>
#include <unordered_map>

// lib include
#include <Eigen/Eigen>
#include <Eigen/Core>

// function include
#include "utility.h"
#include "quatOps.h"
#include "frame.h"

class baseType;
class state;

class updaterHelper
{
public:

	struct updaterHelperFeature
	{
		size_t feature_id;

		std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs;

		std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs_norm;

		std::unordered_map<size_t, std::vector<double>> timestamps;

		std::unordered_map<size_t, std::vector<std::shared_ptr<frame>>> frames;

		std::unordered_map<size_t, std::vector<float>> color;

		int anchor_cam_id = -1;

		double anchor_clone_timestamp = -1;

		Eigen::Vector3d position_anchor;

		Eigen::Vector3d position_anchor_fej;

		Eigen::Vector3d position_global;

		Eigen::Vector3d position_global_fej;
	};

	static void getFeatureJacobianRepresentation(std::shared_ptr<state> state_ptr, updaterHelperFeature &feature, Eigen::MatrixXd &H_f,
		std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<baseType>> &x_order);

	static void getFeatureJacobianFull(std::shared_ptr<state> state_ptr, updaterHelperFeature &feature, Eigen::MatrixXd &H_f, 
		Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<baseType>> &x_order);

	static void nullspaceProjectInplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

	static void measurementCompressInplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);
};
