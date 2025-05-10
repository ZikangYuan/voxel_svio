#pragma once

// c++ include
#include <iostream>
#include <cmath>

// lib include
#include <Eigen/Core>

// function include
#include "utility.h"
#include "quatOps.h"
#include "parameters.h"
#include "frame.h"

class mapPoint;

class baseType
{
public:

	baseType(int size_);

  	virtual void setLocalId(int new_id);

	int getId();

	int getSize();

	virtual void update(const Eigen::VectorXd &dx) = 0;

  	virtual const Eigen::MatrixXd &value() const { return value_; }

	virtual const Eigen::MatrixXd &fej() const { return fej_; }

	virtual void setValue(const Eigen::MatrixXd &new_value);

	virtual void setFej(const Eigen::MatrixXd &new_value);

	virtual std::shared_ptr<baseType> clone() = 0;

	virtual std::shared_ptr<baseType> checkIfSubvariable(const std::shared_ptr<baseType> check);

protected:

	Eigen::MatrixXd fej_;

	Eigen::MatrixXd value_;

	int id = -1;

	int size = -1;
};

class jplQuat : public baseType
{
public:

	jplQuat();

	void update(const Eigen::VectorXd &dx) override;

	void setValue(const Eigen::MatrixXd &new_value) override;

	void setFej(const Eigen::MatrixXd &new_value) override;

	std::shared_ptr<baseType> clone() override;

	Eigen::Matrix<double, 3, 3> getRot();

	Eigen::Matrix<double, 3, 3> getRotFej();

protected:

	Eigen::Matrix<double, 3, 3> R;

	Eigen::Matrix<double, 3, 3> R_fej;

	void setValueInternal(const Eigen::MatrixXd &new_value);

	void setFejInternal(const Eigen::MatrixXd &new_value);
};

class vec : public baseType
{
public:

	vec(int dim);

	void update(const Eigen::VectorXd &dx) override;

	std::shared_ptr<baseType> clone() override;
};

class poseJpl : public baseType
{
public:

	poseJpl();

  	void setLocalId(int new_id) override;

	void update(const Eigen::VectorXd &dx) override;

	void setValue(const Eigen::MatrixXd &new_value) override;

	void setFej(const Eigen::MatrixXd &new_value) override;

	std::shared_ptr<baseType> clone() override;

	std::shared_ptr<baseType> checkIfSubvariable(const std::shared_ptr<baseType> check) override;

	Eigen::Matrix<double, 3, 3> getRot();

	Eigen::Matrix<double, 3, 3> getRotFej();

	Eigen::Matrix<double, 4, 1> getQuat();

	Eigen::Matrix<double, 4, 1> getQuatFej();

	Eigen::Matrix<double, 3, 1> getPos();

	Eigen::Matrix<double, 3, 1> getPosFej();

	std::shared_ptr<jplQuat> q();

	std::shared_ptr<vec> p();

protected:

	std::shared_ptr<jplQuat> q_;

	std::shared_ptr<vec> p_;

	void setValueInternal(const Eigen::MatrixXd &new_value);

	void setFejInternal(const Eigen::MatrixXd &new_value);
};

class imuState : public baseType
{
public:

	imuState();

  	void setLocalId(int new_id) override;

	void update(const Eigen::VectorXd &dx) override;

	void setValue(const Eigen::MatrixXd &new_value) override;

	void setFej(const Eigen::MatrixXd &new_value) override;

	std::shared_ptr<baseType> clone() override;

	std::shared_ptr<baseType> checkIfSubvariable(const std::shared_ptr<baseType> check) override;

	Eigen::Matrix<double, 3, 3> getRot();

	Eigen::Matrix<double, 3, 3> getRotFej();

	Eigen::Matrix<double, 4, 1> getQuat();

	Eigen::Matrix<double, 4, 1> getQuatFej();

	Eigen::Matrix<double, 3, 1> getPos();

	Eigen::Matrix<double, 3, 1> getPosFej();

	Eigen::Matrix<double, 3, 1> getVel();

	Eigen::Matrix<double, 3, 1> getVelFej();

	Eigen::Matrix<double, 3, 1> getBg();

	Eigen::Matrix<double, 3, 1> getBgFej();

	Eigen::Matrix<double, 3, 1> getBa();

	Eigen::Matrix<double, 3, 1> getBaFej();

	std::shared_ptr<poseJpl> pose();

	std::shared_ptr<jplQuat> q();

	std::shared_ptr<vec> p();

	std::shared_ptr<vec> v();

	std::shared_ptr<vec> bg();

	std::shared_ptr<vec> ba();

protected:

	std::shared_ptr<poseJpl> pose_;

	std::shared_ptr<vec> v_;

	std::shared_ptr<vec> bg_;

	std::shared_ptr<vec> ba_;

	void setValueInternal(const Eigen::MatrixXd &new_value);

	void setFejInternal(const Eigen::MatrixXd &new_value);
};

class state
{
public:

	state(stateOptions &options_);

 	double margtimestep();

	int maxCovSize();

	static Eigen::Matrix3d Dm(ImuModel imu_model_, const Eigen::MatrixXd &vec_temp);

	static Eigen::Matrix3d Tg(const Eigen::MatrixXd &vec_temp);

	int imuIntrinsicSize();

	double timestamp = -1; // _timestamp

	stateOptions options;

	std::shared_ptr<imuState> imu_ptr;

	std::map<double, std::shared_ptr<poseJpl>> clones_imu;

	std::map<double, std::shared_ptr<frame>> clones_frame;

	std::unordered_map<size_t, std::shared_ptr<mapPoint>> map_points;

	std::shared_ptr<vec> calib_dt_imu_cam;

	std::unordered_map<size_t, std::shared_ptr<poseJpl>> calib_cam_imu;

	std::unordered_map<size_t, std::shared_ptr<vec>> cam_intrinsics;

	std::unordered_map<size_t, std::shared_ptr<cameraBase>> cam_intrinsics_cameras;

	std::shared_ptr<vec> calib_imu_dw;

	std::shared_ptr<vec> calib_imu_da;

	std::shared_ptr<vec> calib_imu_tg;

	std::shared_ptr<jplQuat> calib_imu_gyr;

	std::shared_ptr<jplQuat> calib_imu_acc;

private:

	friend class stateHelper;

	Eigen::MatrixXd cov;

	std::vector<std::shared_ptr<baseType>> variables;
};