#pragma once

// c++ include
#include <iostream>
#include <cmath>

// lib include
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// function include
#include "utility.h"
#include "mapPoint.h"
#include "mapManagement.h"
#include "feature.h"

class baseType;
class state;

class stateHelper
{
private:

	stateHelper();

public:
	
	static void ekfPropagation(std::shared_ptr<state> state_ptr, const std::vector<std::shared_ptr<baseType>> &order_new, 
		const std::vector<std::shared_ptr<baseType>> &order_old, const Eigen::MatrixXd &phi, const Eigen::MatrixXd &Q);

	static void ekfUpdate(std::shared_ptr<state> state_ptr, const std::vector<std::shared_ptr<baseType>> &H_order, 
		const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R);

	static void setInitialCovariance(std::shared_ptr<state> state_ptr, const Eigen::MatrixXd &covariance, 
		const std::vector<std::shared_ptr<baseType>> &order);

	static Eigen::MatrixXd getMarginalCovariance(std::shared_ptr<state> state_ptr, const std::vector<std::shared_ptr<baseType>> &small_variables);

	static Eigen::MatrixXd getFullCovariance(std::shared_ptr<state> state_ptr);

	static void marginalize(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> marg);

	static std::shared_ptr<baseType> clone(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> variable_to_clone);

	static bool initialize(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> new_variable, 
		const std::vector<std::shared_ptr<baseType>> &H_order, Eigen::MatrixXd &H_R, Eigen::MatrixXd &H_L, 
		Eigen::MatrixXd &R, Eigen::VectorXd &res, double chi_2_mult);

	static void initializeInvertible(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> new_variable, 
		const std::vector<std::shared_ptr<baseType>> &H_order, const Eigen::MatrixXd &H_R, 
		const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res);

	static void augmentClone(std::shared_ptr<state> state_ptr, Eigen::Matrix<double, 3, 1> last_gyr);

	static void marginalizeOldClone(std::shared_ptr<state> state_ptr);

	static void marginalizeNewClone(std::shared_ptr<state> state_ptr);

	static void marginalizeSlam(std::shared_ptr<state> state_ptr, voxelHashMap &voxel_map, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_history);
};
