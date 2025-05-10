#include "state.h"

baseType::baseType(int size_)
{
	size = size_;
}

void baseType::setLocalId(int new_id) 
{
	id = new_id;
}

int baseType::getId()
{
	return id;
}

int baseType::getSize()
{
	return size;
}

void baseType::setValue(const Eigen::MatrixXd &new_value)
{
	assert(value_.rows() == new_value.rows());
	assert(value_.cols() == new_value.cols());
	value_ = new_value;
}

void baseType::setFej(const Eigen::MatrixXd &new_value)
{
	assert(fej_.rows() == new_value.rows());
	assert(fej_.cols() == new_value.cols());
	fej_ = new_value;
}

std::shared_ptr<baseType> baseType::checkIfSubvariable(const std::shared_ptr<baseType> check)
{
	return nullptr;
}

jplQuat::jplQuat() : baseType(3)
{
	Eigen::Vector4d q_temp = Eigen::Vector4d::Zero();
	q_temp(3) = 1.0;
	setValueInternal(q_temp);
	setFejInternal(q_temp);
}

void jplQuat::update(const Eigen::VectorXd &dx)
{
	assert(dx.rows() == size);

	Eigen::Matrix<double, 4, 1> dq;
	dq << 0.5 * dx, 1.0;
	dq = quatType::quatNorm(dq);

	setValue(quatType::quatMultiply(dq, value_));
}

void jplQuat::setValue(const Eigen::MatrixXd &new_value)
{
	setValueInternal(new_value);
}

void jplQuat::setFej(const Eigen::MatrixXd &new_value)
{
	setFejInternal(new_value);
}

std::shared_ptr<baseType> jplQuat::clone()
{
	auto clone_temp = std::shared_ptr<jplQuat>(new jplQuat());
	clone_temp->setValue(value());
	clone_temp->setFej(fej());
	return clone_temp;
}

Eigen::Matrix<double, 3, 3> jplQuat::getRot()
{
	return R;
}

Eigen::Matrix<double, 3, 3> jplQuat::getRotFej()
{
	return R_fej;
}

void jplQuat::setValueInternal(const Eigen::MatrixXd &new_value)
{
	assert(new_value.rows() == 4);
	assert(new_value.cols() == 1);

	value_ = new_value;

	R = quatType::quatToRot(new_value);
}

void jplQuat::setFejInternal(const Eigen::MatrixXd &new_value)
{
	assert(new_value.rows() == 4);
	assert(new_value.cols() == 1);

	fej_ = new_value;

	R_fej = quatType::quatToRot(new_value);
}

vec::vec(int dim) : baseType(dim)
{
	value_ = Eigen::VectorXd::Zero(dim);
	fej_ = Eigen::VectorXd::Zero(dim);
}

void vec::update(const Eigen::VectorXd &dx)
{
	assert(dx.rows() == size);
	setValue(value_ + dx);
}

std::shared_ptr<baseType> vec::clone()
{
	auto clone_temp = std::shared_ptr<baseType>(new vec(size));
	clone_temp->setValue(value());
	clone_temp->setFej(fej());
	return clone_temp;
}

poseJpl::poseJpl() : baseType(6)
{
	q_ = std::shared_ptr<jplQuat>(new jplQuat());
	p_ = std::shared_ptr<vec>(new vec(3));

	Eigen::Matrix<double, 7, 1> pose_temp;
	pose_temp.setZero();
    pose_temp(3) = 1.0;
    setValueInternal(pose_temp);
    setFejInternal(pose_temp);
}

void poseJpl::setLocalId(int new_id)
{
	id = new_id;
	q_->setLocalId(new_id);
	p_->setLocalId(new_id + ((new_id != -1) ? q_->getSize() : 0));
}

void poseJpl::update(const Eigen::VectorXd &dx)
{
	assert(dx.rows() == size);

	Eigen::Matrix<double, 7, 1> newX = value_;
	Eigen::Matrix<double, 4, 1> dq;
	dq << 0.5 * dx.block<3, 1>(0, 0), 1.0;
	dq = quatType::quatNorm(dq);

	newX.block<4, 1>(0, 0) = quatType::quatMultiply(dq, getQuat());
	newX.block<3, 1>(4, 0) += dx.block(3, 0, 3, 1);

	setValue(newX);
}

void poseJpl::setValue(const Eigen::MatrixXd &new_value)
{
	setValueInternal(new_value);
}

void poseJpl::setFej(const Eigen::MatrixXd &new_value)
{
	setFejInternal(new_value);
}

std::shared_ptr<baseType> poseJpl::clone()
{
	auto clone_temp = std::shared_ptr<poseJpl>(new poseJpl());
	clone_temp->setValue(value());
	clone_temp->setFej(fej());
	return clone_temp;
}

std::shared_ptr<baseType> poseJpl::checkIfSubvariable(const std::shared_ptr<baseType> check)
{
	if (check == q_)
	{
		return q_;
	}
	else if (check == p_)
	{
		return p_;
	}

	return nullptr;
}

Eigen::Matrix<double, 3, 3> poseJpl::getRot()
{
	return q_->getRot();
}

Eigen::Matrix<double, 3, 3> poseJpl::getRotFej()
{
	return q_->getRotFej();
}

Eigen::Matrix<double, 4, 1> poseJpl::getQuat()
{
	return q_->value();
}

Eigen::Matrix<double, 4, 1> poseJpl::getQuatFej()
{
	return q_->fej();
}

Eigen::Matrix<double, 3, 1> poseJpl::getPos()
{
	return p_->value();
}

Eigen::Matrix<double, 3, 1> poseJpl::getPosFej()
{
	return p_->fej();
}

std::shared_ptr<jplQuat> poseJpl::q()
{
	return q_;
}

std::shared_ptr<vec> poseJpl::p()
{
	return p_;
}

void poseJpl::setValueInternal(const Eigen::MatrixXd &new_value)
{
	assert(new_value.rows() == 7);
	assert(new_value.cols() == 1);

	q_->setValue(new_value.block<4, 1>(0, 0));
	p_->setValue(new_value.block<3, 1>(4, 0));

	value_ = new_value;
}

void poseJpl::setFejInternal(const Eigen::MatrixXd &new_value)
{
	assert(new_value.rows() == 7);
	assert(new_value.cols() == 1);

	q_->setFej(new_value.block<4, 1>(0, 0));
	p_->setFej(new_value.block<3, 1>(4, 0));

	fej_ = new_value;
}

imuState::imuState() : baseType(15)
{
	pose_ = std::shared_ptr<poseJpl>(new poseJpl());
	v_ = std::shared_ptr<vec>(new vec(3));
	bg_ = std::shared_ptr<vec>(new vec(3));
	ba_ = std::shared_ptr<vec>(new vec(3));

	Eigen::VectorXd imu_temp = Eigen::VectorXd::Zero(16, 1);
	imu_temp(3) = 1.0;
	setValueInternal(imu_temp);
	setFejInternal(imu_temp);
}

void imuState::setLocalId(int new_id)
{
	id = new_id;
	pose_->setLocalId(new_id);
	v_->setLocalId(pose_->getId() + ((new_id != -1) ? pose_->getSize() : 0));
	bg_->setLocalId(v_->getId() + ((new_id != -1) ? v_->getSize() : 0));
	ba_->setLocalId(bg_->getId() + ((new_id != -1) ? bg_->getSize() : 0));
}

void imuState::update(const Eigen::VectorXd &dx)
{
	assert(dx.rows() == size);

	Eigen::Matrix<double, 16, 1> new_x = value_;

	Eigen::Matrix<double, 4, 1> dq;
	dq << 0.5 * dx.block<3, 1>(0, 0), 1.0;
	dq = quatType::quatNorm(dq);

	new_x.block<4, 1>(0, 0) = quatType::quatMultiply(dq, getQuat());
	new_x.block<3, 1>(4, 0) += dx.block<3, 1>(3, 0);

	new_x.block<3, 1>(7, 0) += dx.block<3, 1>(6, 0);
	new_x.block<3, 1>(10, 0) += dx.block<3, 1>(9, 0);
	new_x.block<3, 1>(13, 0) += dx.block<3, 1>(12, 0);

	setValue(new_x);
}

void imuState::setValue(const Eigen::MatrixXd &new_value)
{
	setValueInternal(new_value);
}

void imuState::setFej(const Eigen::MatrixXd &new_value)
{
	setFejInternal(new_value);
}

std::shared_ptr<baseType> imuState::clone()
{
	auto clone_temp = std::shared_ptr<baseType>(new imuState());
	clone_temp->setValue(value());
	clone_temp->setFej(fej());
	return clone_temp;
}

std::shared_ptr<baseType> imuState::checkIfSubvariable(const std::shared_ptr<baseType> check)
{
	if (check == pose_)
	{
		return pose_;
	}
	else if (check == pose_->checkIfSubvariable(check))
	{
		return pose_->checkIfSubvariable(check);
	} 
	else if (check == v_)
	{
		return v_;
	}
	else if (check == bg_)
	{
		return bg_;
	}
	else if (check == ba_)
	{
		return ba_;
	}
	return nullptr;
}

Eigen::Matrix<double, 3, 3> imuState::getRot()
{
	return pose_->getRot();
}

Eigen::Matrix<double, 3, 3> imuState::getRotFej()
{
	return pose_->getRotFej();
}

Eigen::Matrix<double, 4, 1> imuState::getQuat()
{
	return pose_->getQuat();
}

Eigen::Matrix<double, 4, 1> imuState::getQuatFej()
{
	return pose_->getQuatFej();
}

Eigen::Matrix<double, 3, 1> imuState::getPos()
{
	return pose_->getPos();
}

Eigen::Matrix<double, 3, 1> imuState::getPosFej()
{
	return pose_->getPosFej();
}

Eigen::Matrix<double, 3, 1> imuState::getVel()
{
	return v_->value();
}

Eigen::Matrix<double, 3, 1> imuState::getVelFej()
{
	return v_->fej();
}

Eigen::Matrix<double, 3, 1> imuState::getBg()
{
	return bg_->value();
}

Eigen::Matrix<double, 3, 1> imuState::getBgFej()
{
	return bg_->fej();
}

Eigen::Matrix<double, 3, 1> imuState::getBa()
{
	return ba_->value();
}

Eigen::Matrix<double, 3, 1> imuState::getBaFej()
{
	return ba_->fej();
}

std::shared_ptr<poseJpl> imuState::pose()
{
	return pose_;
}

std::shared_ptr<jplQuat> imuState::q()
{
	return pose_->q();
}

std::shared_ptr<vec> imuState::p()
{
	return pose_->p();
}

std::shared_ptr<vec> imuState::v()
{
	return v_;
}

std::shared_ptr<vec> imuState::bg()
{
	return bg_;
}

std::shared_ptr<vec> imuState::ba()
{
	return ba_;
}

void imuState::setValueInternal(const Eigen::MatrixXd &new_value)
{
	assert(new_value.rows() == 16);
	assert(new_value.cols() == 1);

	pose_->setValue(new_value.block<7, 1>(0, 0));
	v_->setValue(new_value.block<3, 1>(7, 0));
	bg_->setValue(new_value.block<3, 1>(10, 0));
	ba_->setValue(new_value.block<3, 1>(13, 0));

	value_ = new_value;
}

void imuState::setFejInternal(const Eigen::MatrixXd &new_value)
{
	assert(new_value.rows() == 16);
	assert(new_value.cols() == 1);

	pose_->setFej(new_value.block<7, 1>(0, 0));
	v_->setFej(new_value.block<3, 1>(7, 0));
	bg_->setFej(new_value.block<3, 1>(10, 0));
	ba_->setFej(new_value.block<3, 1>(13, 0));

	fej_ = new_value;
}

state::state(stateOptions &options_)
{
	options = options_;

	int current_id = 0;
	imu_ptr = std::make_shared<imuState>();
	imu_ptr->setLocalId(current_id);
	variables.push_back(imu_ptr);
	current_id += imu_ptr->getSize();

	calib_imu_dw = std::make_shared<vec>(6);
	calib_imu_da = std::make_shared<vec>(6);

	if (options.imu_model == ImuModel::KALIBR)
	{
    Eigen::Matrix<double, 6, 1> imu_default = Eigen::Matrix<double, 6, 1>::Zero();
    imu_default << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0;
    calib_imu_dw->setValue(imu_default);
    calib_imu_dw->setFej(imu_default);
    calib_imu_da->setValue(imu_default);
    calib_imu_da->setFej(imu_default);
  }
  else
  {
    Eigen::Matrix<double, 6, 1> imu_default = Eigen::Matrix<double, 6, 1>::Zero();
    imu_default << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0;
    calib_imu_dw->setValue(imu_default);
    calib_imu_dw->setFej(imu_default);
    calib_imu_da->setValue(imu_default);
    calib_imu_da->setFej(imu_default);
  }
  calib_imu_tg = std::make_shared<vec>(9);
  calib_imu_gyr = std::make_shared<jplQuat>();
  calib_imu_acc = std::make_shared<jplQuat>();

  if (options.do_calib_imu_intrinsics)
  {
    calib_imu_dw->setLocalId(current_id);
    variables.push_back(calib_imu_dw);
    current_id += calib_imu_dw->getSize();

    calib_imu_da->setLocalId(current_id);
    variables.push_back(calib_imu_da);
    current_id += calib_imu_da->getSize();

    if (options.do_calib_imu_g_sensitivity)
    {
      calib_imu_tg->setLocalId(current_id);
      variables.push_back(calib_imu_tg);
      current_id += calib_imu_tg->getSize();
    }

    if (options.imu_model == ImuModel::KALIBR)
    {
      calib_imu_gyr->setLocalId(current_id);
      variables.push_back(calib_imu_gyr);
      current_id += calib_imu_gyr->getSize();
    }
    else
    {
      calib_imu_acc->setLocalId(current_id);
      variables.push_back(calib_imu_acc);
      current_id += calib_imu_acc->getSize();
    }
  }

  calib_dt_imu_cam = std::make_shared<vec>(1);

  if (options.do_calib_camera_timeoffset)
  {
    calib_dt_imu_cam->setLocalId(current_id);
    variables.push_back(calib_dt_imu_cam);
    current_id += calib_dt_imu_cam->getSize();
  }

  for (int i = 0; i < 2; i++)
  {
    auto pose = std::make_shared<poseJpl>();
    auto intrin = std::make_shared<vec>(8);

    calib_cam_imu.insert({i, pose});
    cam_intrinsics.insert({i, intrin});

    if (options.do_calib_camera_pose)
    {
      pose->setLocalId(current_id);
      variables.push_back(pose);
      current_id += pose->getSize();
    }

    if (options.do_calib_camera_intrinsics)
    {
      intrin->setLocalId(current_id);
      variables.push_back(intrin);
      current_id += intrin->getSize();
    }
  }

  cov = std::pow(1e-3, 2) * Eigen::MatrixXd::Identity(current_id, current_id);

  if (options.do_calib_imu_intrinsics)
  {
    cov.block(calib_imu_dw->getId(), calib_imu_dw->getId(), 6, 6) = std::pow(0.005, 2) * Eigen::Matrix<double, 6, 6>::Identity();
    cov.block(calib_imu_da->getId(), calib_imu_da->getId(), 6, 6) = std::pow(0.008, 2) * Eigen::Matrix<double, 6, 6>::Identity();

    if (options.do_calib_imu_g_sensitivity)
      cov.block(calib_imu_tg->getId(), calib_imu_tg->getId(), 9, 9) = std::pow(0.005, 2) * Eigen::Matrix<double, 9, 9>::Identity();

    if (options.imu_model == ImuModel::KALIBR)
      cov.block(calib_imu_gyr->getId(), calib_imu_gyr->getId(), 3, 3) = std::pow(0.005, 2) * Eigen::Matrix3d::Identity();
    else
      cov.block(calib_imu_acc->getId(), calib_imu_acc->getId(), 3, 3) = std::pow(0.005, 2) * Eigen::Matrix3d::Identity();
  }

  if (options.do_calib_camera_timeoffset)
    cov(calib_dt_imu_cam->getId(), calib_dt_imu_cam->getId()) = std::pow(0.01, 2);

  if (options.do_calib_camera_pose)
  {
    for (int i = 0; i < 2; i++)
    {
      cov.block(calib_cam_imu.at(i)->getId(), calib_cam_imu.at(i)->getId(), 3, 3) = std::pow(0.005, 2) * Eigen::MatrixXd::Identity(3, 3);
      cov.block(calib_cam_imu.at(i)->getId() + 3, calib_cam_imu.at(i)->getId() + 3, 3, 3) = std::pow(0.015, 2) * Eigen::MatrixXd::Identity(3, 3);
    }
  }

  if (options.do_calib_camera_intrinsics)
  {
    for (int i = 0; i < 2; i++) {
      cov.block(cam_intrinsics.at(i)->getId(), cam_intrinsics.at(i)->getId(), 4, 4) = std::pow(1.0, 2) * Eigen::MatrixXd::Identity(4, 4);
      cov.block(cam_intrinsics.at(i)->getId() + 4, cam_intrinsics.at(i)->getId() + 4, 4, 4) = std::pow(0.005, 2) * Eigen::MatrixXd::Identity(4, 4);
    }
  }
}

double state::margtimestep()
{
	double time = INFINITY;

	for (const auto &clone_imu : clones_imu)
	{
		if (clone_imu.first < time)
		{
			time = clone_imu.first;
		}
	}

  return time;
}

int state::maxCovSize()
{
	return (int)cov.rows();
}

Eigen::Matrix3d state::Dm(ImuModel imu_model_, const Eigen::MatrixXd &vec_temp)
{
	assert(vec_temp.rows() == 6);
	assert(vec_temp.cols() == 1);
	Eigen::Matrix3d D_matrix = Eigen::Matrix3d::Identity();
	
	if (imu_model_ == ImuModel::KALIBR)
		D_matrix << vec_temp(0), 0, 0, vec_temp(1), vec_temp(3), 0, vec_temp(2), vec_temp(4), vec_temp(5);
	else
		D_matrix << vec_temp(0), vec_temp(1), vec_temp(3), 0, vec_temp(2), vec_temp(4), 0, 0, vec_temp(5);

	return D_matrix;
}

Eigen::Matrix3d state::Tg(const Eigen::MatrixXd &vec_temp)
{
	assert(vec_temp.rows() == 9);
	assert(vec_temp.cols() == 1);
	Eigen::Matrix3d Tg = Eigen::Matrix3d::Zero();

	Tg << vec_temp(0), vec_temp(3), vec_temp(6), vec_temp(1), vec_temp(4), vec_temp(7), vec_temp(2), vec_temp(5), vec_temp(8);
	
	return Tg;
}

int state::imuIntrinsicSize()
{
	int size_temp = 0;

	if (options.do_calib_imu_intrinsics)
	{
		size_temp += 15;

		if (options.do_calib_imu_g_sensitivity)
			size_temp += 9;
	}
	
	return size_temp;
}