#pragma once

// c++ include
#include <iostream>
#include <math.h>

// lib include
#include <Eigen/Core>

class quatType
{
public:
	static Eigen::Matrix<double, 4, 1> rotToQuat(const Eigen::Matrix<double, 3, 3> &rot);

	static Eigen::Matrix<double, 3, 3> quatToRot(const Eigen::Matrix<double, 4, 1> &q);

	static Eigen::Matrix<double, 3, 3> skewSymmetric(const Eigen::Matrix<double, 3, 1> &w);

	static Eigen::Matrix<double, 4, 1> quatMultiply(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p);

	static Eigen::Matrix<double, 3, 1> vee(const Eigen::Matrix<double, 3, 3> &w_x);

	static Eigen::Matrix<double, 3, 3> expSo3(const Eigen::Matrix<double, 3, 1> &w);

	static Eigen::Matrix<double, 3, 1> logSo3(const Eigen::Matrix<double, 3, 3> &R);

	static Eigen::Matrix4d expSe3(Eigen::Matrix<double, 6, 1> vec);

	static Eigen::Matrix<double, 6, 1> logSe3(Eigen::Matrix4d mat);

	static Eigen::Matrix4d hatSe3(const Eigen::Matrix<double, 6, 1> &vec);

	static Eigen::Matrix4d invSe3(const Eigen::Matrix4d &T);

	static Eigen::Matrix<double, 4, 1> inv(Eigen::Matrix<double, 4, 1> q);

	static Eigen::Matrix<double, 4, 4> omega(Eigen::Matrix<double, 3, 1> w);

	static Eigen::Matrix<double, 4, 1> quatNorm(Eigen::Matrix<double, 4, 1> q_t);

	static Eigen::Matrix<double, 3, 3> JleftSo3(const Eigen::Matrix<double, 3, 1> &w);

	static Eigen::Matrix<double, 3, 3> JrighySo3(const Eigen::Matrix<double, 3, 1> &w);

	static Eigen::Matrix<double, 3, 1> rotToRpy(const Eigen::Matrix<double, 3, 3> &rot);

	static Eigen::Matrix<double, 3, 3> rotX(double t);

	static Eigen::Matrix<double, 3, 3> rotY(double t);

	static Eigen::Matrix<double, 3, 3> rotZ(double t);
};