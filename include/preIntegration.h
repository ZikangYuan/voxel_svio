#pragma once

// c++ include
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// lib include
#include <Eigen/Core>
#include <ceres/ceres.h>

// function include
#include "quatOps.h"
#include "camera.h"

class preIntegrationBase
{
public:

	preIntegrationBase(double sigma_w_, double sigma_wb_, double sigma_a_, double sigma_ab_, bool imu_avg_ = false);

	virtual ~preIntegrationBase() {}

	void setLinearizationPoints(Eigen::Matrix<double, 3, 1> b_w_lin_, Eigen::Matrix<double, 3, 1> b_a_lin_,
		Eigen::Matrix<double, 4, 1> q_k_lin_ = Eigen::Matrix<double, 4, 1>::Zero(), Eigen::Matrix<double, 3, 1> grav_ = Eigen::Matrix<double, 3, 1>::Zero());

	virtual void feedImu(double t_0, double t_1, Eigen::Matrix<double, 3, 1> w_m_0, Eigen::Matrix<double, 3, 1> a_m_0, 
		Eigen::Matrix<double, 3, 1> w_m_1 = Eigen::Matrix<double, 3, 1>::Zero(), Eigen::Matrix<double, 3, 1> a_m_1 = Eigen::Matrix<double, 3, 1>::Zero()) = 0;

	bool imu_avg = false;

	double dt = 0;
	Eigen::Matrix<double, 3, 1> alpha_tau = Eigen::Matrix<double, 3, 1>::Zero();
	Eigen::Matrix<double, 3, 1> beta_tau = Eigen::Matrix<double, 3, 1>::Zero();
	Eigen::Matrix<double, 4, 1> q_tau_k;
	Eigen::Matrix<double, 3, 3> R_tau_k = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 3, 3> J_q = Eigen::Matrix<double, 3, 3>::Zero();
	Eigen::Matrix<double, 3, 3> J_a = Eigen::Matrix<double, 3, 3>::Zero();
	Eigen::Matrix<double, 3, 3> J_b = Eigen::Matrix<double, 3, 3>::Zero();
	Eigen::Matrix<double, 3, 3> H_a = Eigen::Matrix<double, 3, 3>::Zero();
	Eigen::Matrix<double, 3, 3> H_b = Eigen::Matrix<double, 3, 3>::Zero();

	Eigen::Matrix<double, 3, 1> b_w_lin;
	Eigen::Matrix<double, 3, 1> b_a_lin;
	Eigen::Matrix<double, 4, 1> q_k_lin;

	Eigen::Matrix<double, 3, 1> grav = Eigen::Matrix<double, 3, 1>::Zero();

	Eigen::Matrix<double, 12, 12> Q_c = Eigen::Matrix<double, 12, 12>::Zero();

	Eigen::Matrix<double, 15, 15> P_meas = Eigen::Matrix<double, 15, 15>::Zero();

	Eigen::Matrix<double, 3, 1> e_1;
	Eigen::Matrix<double, 3, 1> e_2;
	Eigen::Matrix<double, 3, 1> e_3;

	Eigen::Matrix<double, 3, 3> e_1x;
	Eigen::Matrix<double, 3, 3> e_2x;
	Eigen::Matrix<double, 3, 3> e_3x;
};

class preIntegrationV1 : public preIntegrationBase
{
public:

	preIntegrationV1(double sigma_w_, double sigma_wb_, double sigma_a_, double sigma_ab_, bool imu_avg_ = false) 
		: preIntegrationBase(sigma_w_, sigma_wb_, sigma_a_, sigma_ab_, imu_avg_) {}

	virtual ~preIntegrationV1() {}

	void feedImu(double t_0, double t_1, Eigen::Matrix<double, 3, 1> w_m_0, Eigen::Matrix<double, 3, 1> a_m_0, 
		Eigen::Matrix<double, 3, 1> w_m_1 = Eigen::Matrix<double, 3, 1>::Zero(), Eigen::Matrix<double, 3, 1> a_m_1 = Eigen::Matrix<double, 3, 1>::Zero());
};

class preIntegrationV2 : public preIntegrationBase
{
private:

	Eigen::Matrix<double, 21, 21> P_big = Eigen::Matrix<double, 21, 21>::Zero();

	Eigen::Matrix<double, 21, 21> Discrete_J_b = Eigen::Matrix<double, 21, 21>::Identity();

public:

	bool state_transition_jacobians = true;

	Eigen::Matrix<double, 3, 3> O_a = Eigen::Matrix<double, 3, 3>::Zero();
	Eigen::Matrix<double, 3, 3> O_b = Eigen::Matrix<double, 3, 3>::Zero();

	preIntegrationV2(double sigma_w_, double sigma_wb_, double sigma_a_, double sigma_ab_, bool imu_avg_ = false) 
		: preIntegrationBase(sigma_w_, sigma_wb_, sigma_a_, sigma_ab_, imu_avg_) {}

	virtual ~preIntegrationV2() {}

	void feedImu(double t_0, double t_1, Eigen::Matrix<double, 3, 1> w_m_0, Eigen::Matrix<double, 3, 1> a_m_0, 
		Eigen::Matrix<double, 3, 1> w_m_1 = Eigen::Matrix<double, 3, 1>::Zero(), Eigen::Matrix<double, 3, 1> a_m_1 = Eigen::Matrix<double, 3, 1>::Zero());
};

class stateJplQuatLocal : public ceres::LocalParameterization
{
public:

	bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;

	bool ComputeJacobian(const double *x, double *jacobian) const override;

	int GlobalSize() const override { return 4; };

	int LocalSize() const override { return 3; };
};

class factorGenericPrior : public ceres::CostFunction
{
public:

	Eigen::MatrixXd x_lin;

	std::vector<std::string> x_type;

	Eigen::MatrixXd sqrt_I;

	Eigen::MatrixXd b;

	factorGenericPrior(const Eigen::MatrixXd &x_lin_, const std::vector<std::string> &x_type_, const Eigen::MatrixXd &prior_Info, 
		const Eigen::MatrixXd &prior_grad);

	virtual ~factorGenericPrior() {}

	bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

class factorImuPreIntegrationV1 : public ceres::CostFunction
{
public:

	Eigen::Vector3d alpha;
	Eigen::Vector3d beta;
	Eigen::Vector4d q_breve;
	double dt;

	Eigen::Vector3d b_w_lin_save;
	Eigen::Vector3d b_a_lin_save;

	Eigen::Matrix3d J_q;
	Eigen::Matrix3d J_a;
	Eigen::Matrix3d J_b;
	Eigen::Matrix3d H_a;
	Eigen::Matrix3d H_b;

	Eigen::Matrix<double, 15, 15> sqrt_I_save;

	Eigen::Vector3d grav_save;

	factorImuPreIntegrationV1(double deltatime, Eigen::Vector3d &grav, Eigen::Vector3d &alpha, Eigen::Vector3d &beta, Eigen::Vector4d &q_KtoK1,
		Eigen::Vector3d &ba_lin, Eigen::Vector3d &bg_lin, Eigen::Matrix3d &J_q, Eigen::Matrix3d &J_beta, Eigen::Matrix3d &J_alpha,
		Eigen::Matrix3d &H_beta, Eigen::Matrix3d &H_alpha, Eigen::Matrix<double, 15, 15> &covariance);

	virtual ~factorImuPreIntegrationV1() {}

	bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

class factorImageReprojCalib : public ceres::CostFunction
{
public:

	Eigen::Vector2d uv_meas;

	double pix_sigma = 1.0;
	Eigen::Matrix<double, 2, 2> sqrt_Q;

	bool is_fisheye = false;

	double gate = 1.0;

	factorImageReprojCalib(const Eigen::Vector2d &uv_meas_, double pix_sigma_, bool is_fisheye_);

	virtual ~factorImageReprojCalib() {}

	bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};