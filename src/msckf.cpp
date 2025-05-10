#include "msckf.h"
#include "state.h"
#include "stateHelper.h"
#include "mapManagement.h"

propagator::propagator(noiseManager noises_, double gravity_mag_) : noises(noises_), cache_imu_valid(false)
{
	noises.sigma_w_2 = std::pow(noises.sigma_w, 2);
	noises.sigma_a_2 = std::pow(noises.sigma_a, 2);
	noises.sigma_wb_2 = std::pow(noises.sigma_wb, 2);
	noises.sigma_ab_2 = std::pow(noises.sigma_ab, 2);

	gravity << 0.0, 0.0, gravity_mag_;

	have_last_prop_time_offset = false;
	last_prop_time_offset = 0.0;
}

void propagator::feedImu(const imuData &imu_data, double oldest_time)
{
	v_imu_data.emplace_back(imu_data);

	cleanOldImuMeasurements(oldest_time - 0.10);	
}

void propagator::cleanOldImuMeasurements(double oldest_time)
{
	if (oldest_time < 0)
		return;
	auto it0 = v_imu_data.begin();
	while (it0 != v_imu_data.end())
	{
		if (it0->timestamp < oldest_time)
		{
			it0 = v_imu_data.erase(it0);
		}
		else
		{
			it0++;
		}
	}
}

void propagator::invalidateCache()
{
	cache_imu_valid = false;
}

imuData propagator::interpolateData(const imuData &imu_1, const imuData &imu_2, double timestamp)
{
	double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);

	imuData data;
	data.timestamp = timestamp;
	data.acc = (1 - lambda) * imu_1.acc + lambda * imu_2.acc;
	data.gyr = (1 - lambda) * imu_1.gyr + lambda * imu_2.gyr;

	return data;
}

std::vector<imuData> propagator::selectImuReadings(const std::vector<imuData> &v_imu_data_, double time_0, double time_1, bool warn)
{
	std::vector<imuData> prop_data;

	if (v_imu_data_.empty())
	{
		if (warn)
			std::cout << "propagator::selectImuReadings(): No IMU measurements. IMU-CAMERA are likely messed up" << std::endl;
		
		return prop_data;
	}

	for (size_t i = 0; i < v_imu_data_.size() - 1; i++)
	{
		if (v_imu_data_.at(i + 1).timestamp > time_0 && v_imu_data_.at(i).timestamp < time_0)
		{
			imuData data = propagator::interpolateData(v_imu_data_.at(i), v_imu_data_.at(i + 1), time_0);
			prop_data.push_back(data);

			continue;
		}

		if (v_imu_data_.at(i).timestamp >= time_0 && v_imu_data_.at(i + 1).timestamp <= time_1)
		{
			prop_data.push_back(v_imu_data_.at(i));
			continue;
		}

		if (v_imu_data_.at(i + 1).timestamp > time_1)
		{
			if (v_imu_data_.at(i).timestamp > time_1 && i == 0)
			{
				break;
			}
			else if (v_imu_data_.at(i).timestamp > time_1)
			{
				imuData data = propagator::interpolateData(v_imu_data_.at(i - 1), v_imu_data_.at(i), time_1);
				prop_data.push_back(data);
			}
			else
			{
				prop_data.push_back(v_imu_data_.at(i));
			}

			if (prop_data.at(prop_data.size() - 1).timestamp != time_1)
			{
				imuData data = propagator::interpolateData(v_imu_data_.at(i), v_imu_data_.at(i + 1), time_1);
				prop_data.push_back(data);
			}

			break;
		}
	}

	if (prop_data.empty())
	{
		if (warn)
			std::cout << "propagator::selectImuReadings(): No IMU measurements to propagate with (" << (int)prop_data.size() << " of 2). IMU-CAMERA are likely messed up" << std::endl;

		return prop_data;
	}

	if (prop_data.at(prop_data.size() - 1).timestamp != time_1)
	{
		if (warn)
			std::cout << std::fixed << "propagator::selectImuReadings(): Missing inertial measurements to propagate with (" << time_1 - v_imu_data_.at(v_imu_data_.size() - 1).timestamp 
				<< " sec missing)" << std::endl;

		imuData data = propagator::interpolateData(v_imu_data_.at(v_imu_data_.size() - 2), v_imu_data_.at(v_imu_data_.size() - 1), time_1);
		prop_data.push_back(data);
	}

	for (size_t i = 0; i < prop_data.size() - 1; i++)
	{
		if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) < 1e-12)
		{
			if (warn)
				std::cout << "propagator::selectImuReadings(): Zero DT between IMU reading " << (int)i << " and " << (int)(i + 1) << ", removing it" << std::endl;

			prop_data.erase(prop_data.begin() + i);
			i--;
		}
	}

	if (prop_data.size() < 2)
	{
		if (warn)
			std::cout << "propagator::selectImuReadings(): No IMU measurements to propagate with (" << (int)prop_data.size() << " of 2). IMU-CAMERA are likely messed up" << std::endl;

		return prop_data;
	}

	return prop_data;
}

void propagator::propagateAndClone(std::shared_ptr<state> state_ptr, double timestamp)
{
	if (state_ptr->timestamp == timestamp)
	{
		std::cout << "propagator::propagateAndClone(): Propagation called again at same timestep at last update timestep!" << std::endl;
		std::exit(EXIT_FAILURE);
	}

	if (state_ptr->timestamp > timestamp)
	{
		std::cout << "propagator::propagateAndClone(): Propagation called trying to propagate backwards in time!" << std::endl;
		std::cout << std::fixed << "propagator::propagateAndClone(): desired propagation = " << timestamp - state_ptr->timestamp << std::endl;
		std::exit(EXIT_FAILURE);
	}

	if (!have_last_prop_time_offset)
	{
		last_prop_time_offset = state_ptr->calib_dt_imu_cam->value()(0);
		have_last_prop_time_offset = true;
	}

	double t_off_new = state_ptr->calib_dt_imu_cam->value()(0);

	double time_0 = state_ptr->timestamp + last_prop_time_offset;
	double time_1 = timestamp + t_off_new;
	std::vector<imuData> prop_data;
	{
		prop_data = propagator::selectImuReadings(v_imu_data, time_0, time_1);
	}

	Eigen::MatrixXd phi_sum = Eigen::MatrixXd::Identity(state_ptr->imuIntrinsicSize() + 15, state_ptr->imuIntrinsicSize() + 15);
	Eigen::MatrixXd Qd_sum = Eigen::MatrixXd::Zero(state_ptr->imuIntrinsicSize() + 15, state_ptr->imuIntrinsicSize() + 15);
	double dt_sum = 0;

	if (prop_data.size() > 1)
	{
		for (size_t i = 0; i < prop_data.size() - 1; i++)
		{
			Eigen::MatrixXd F, Qdi;
			predictAndCompute(state_ptr, prop_data.at(i), prop_data.at(i + 1), F, Qdi);

			phi_sum = F * phi_sum;
			Qd_sum = F * Qd_sum * F.transpose() + Qdi;
			Qd_sum = 0.5 * (Qd_sum + Qd_sum.transpose());
			dt_sum += prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp;
		}
	}
	assert(std::abs((time_1 - time_0) - dt_sum) < 1e-4);

	Eigen::Vector3d last_acc = Eigen::Vector3d::Zero();
	Eigen::Vector3d last_gyr = Eigen::Vector3d::Zero();
	if (!prop_data.empty())
	{
		Eigen::Matrix3d Dw = state::Dm(state_ptr->options.imu_model, state_ptr->calib_imu_dw->value());
		Eigen::Matrix3d Da = state::Dm(state_ptr->options.imu_model, state_ptr->calib_imu_da->value());
		Eigen::Matrix3d Tg = state::Tg(state_ptr->calib_imu_tg->value());
		last_acc = state_ptr->calib_imu_acc->getRot() * Da * (prop_data.at(prop_data.size() - 1).acc - state_ptr->imu_ptr->getBa());
		last_gyr = state_ptr->calib_imu_gyr->getRot() * Dw * (prop_data.at(prop_data.size() - 1).gyr - state_ptr->imu_ptr->getBg() - Tg * last_acc);
	}

	std::vector<std::shared_ptr<baseType>> phi_order;
	phi_order.push_back(state_ptr->imu_ptr);
	
	if (state_ptr->options.do_calib_imu_intrinsics)
	{
		phi_order.push_back(state_ptr->calib_imu_dw);
		phi_order.push_back(state_ptr->calib_imu_da);

		if (state_ptr->options.do_calib_imu_g_sensitivity)
		{
			phi_order.push_back(state_ptr->calib_imu_tg);
		}

		if (state_ptr->options.imu_model == ImuModel::KALIBR)
		{
			phi_order.push_back(state_ptr->calib_imu_gyr);
		}
		else
		{
			phi_order.push_back(state_ptr->calib_imu_acc);
		}
	}

	stateHelper::ekfPropagation(state_ptr, phi_order, phi_order, phi_sum, Qd_sum);

	state_ptr->timestamp = timestamp;
	last_prop_time_offset = t_off_new;

	stateHelper::augmentClone(state_ptr, last_gyr);
}

void propagator::predictAndCompute(std::shared_ptr<state> state_ptr, const imuData &data_minus, const imuData &data_plus, 
	Eigen::MatrixXd &F, Eigen::MatrixXd &Qd)
{
	double dt = data_plus.timestamp - data_minus.timestamp;

	Eigen::Matrix3d Dw = state::Dm(state_ptr->options.imu_model, state_ptr->calib_imu_dw->value());
	Eigen::Matrix3d Da = state::Dm(state_ptr->options.imu_model, state_ptr->calib_imu_da->value());
	Eigen::Matrix3d Tg = state::Tg(state_ptr->calib_imu_tg->value());

	Eigen::Vector3d a_hat1 = data_minus.acc - state_ptr->imu_ptr->getBa();
	Eigen::Vector3d a_hat2 = data_plus.acc - state_ptr->imu_ptr->getBa();
	Eigen::Vector3d a_hat_avg = 0.5 * (a_hat1 + a_hat2);

	Eigen::Vector3d a_uncorrected = a_hat_avg;
	Eigen::Matrix3d R_imu_acc = state_ptr->calib_imu_acc->getRot();
	a_hat1 = R_imu_acc * Da * a_hat1;
	a_hat2 = R_imu_acc * Da * a_hat2;
	a_hat_avg = R_imu_acc * Da * a_hat_avg;

	Eigen::Vector3d w_hat1 = data_minus.gyr - state_ptr->imu_ptr->getBg() - Tg * a_hat1;
	Eigen::Vector3d w_hat2 = data_plus.gyr - state_ptr->imu_ptr->getBg() - Tg * a_hat2;
	Eigen::Vector3d w_hat_avg = 0.5 * (w_hat1 + w_hat2);

	Eigen::Vector3d w_uncorrected = w_hat_avg;
	Eigen::Matrix3d R_imu_gyr = state_ptr->calib_imu_gyr->getRot();
	w_hat1 = R_imu_gyr * Dw * w_hat1;
	w_hat2 = R_imu_gyr * Dw * w_hat2;
	w_hat_avg = R_imu_gyr * Dw * w_hat_avg;

	Eigen::Matrix<double, 3, 18> Xi_sum = Eigen::Matrix<double, 3, 18>::Zero(3, 18);
	computeXiSum(state_ptr, dt, w_hat_avg, a_hat_avg, Xi_sum);

	Eigen::Vector4d new_q;
	Eigen::Vector3d new_v, new_p;
	predictMeanRk4(state_ptr, dt, w_hat1, a_hat1, w_hat2, a_hat2, new_q, new_v, new_p);

	F = Eigen::MatrixXd::Zero(state_ptr->imuIntrinsicSize() + 15, state_ptr->imuIntrinsicSize() + 15);
	Eigen::MatrixXd G = Eigen::MatrixXd::Zero(state_ptr->imuIntrinsicSize() + 15, 12);
	computeCoefficientAnalytic(state_ptr, dt, w_hat_avg, a_hat_avg, w_uncorrected, a_uncorrected, new_q, new_v, new_p, Xi_sum, F, G);

	Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
	Qc.block(0, 0, 3, 3) = std::pow(noises.sigma_w, 2) / dt * Eigen::Matrix3d::Identity();
	Qc.block(3, 3, 3, 3) = std::pow(noises.sigma_a, 2) / dt * Eigen::Matrix3d::Identity();
	Qc.block(6, 6, 3, 3) = std::pow(noises.sigma_wb, 2) / dt * Eigen::Matrix3d::Identity();
	Qc.block(9, 9, 3, 3) = std::pow(noises.sigma_ab, 2) / dt * Eigen::Matrix3d::Identity();

	Qd = Eigen::MatrixXd::Zero(state_ptr->imuIntrinsicSize() + 15, state_ptr->imuIntrinsicSize() + 15);
	Qd = G * Qc * G.transpose();
	Qd = 0.5 * (Qd + Qd.transpose());

	Eigen::Matrix<double, 16, 1> imu_x = state_ptr->imu_ptr->value();
	imu_x.block(0, 0, 4, 1) = new_q;
	imu_x.block(4, 0, 3, 1) = new_p;
	imu_x.block(7, 0, 3, 1) = new_v;
	state_ptr->imu_ptr->setValue(imu_x);
	state_ptr->imu_ptr->setFej(imu_x);
}

void propagator::computeXiSum(std::shared_ptr<state> state_ptr, double dt, const Eigen::Vector3d &un_gyr, 
	const Eigen::Vector3d &un_acc, Eigen::Matrix<double, 3, 18> &Xi_sum)
{
	double un_gyr_norm = un_gyr.norm();
	double d_th = un_gyr_norm * dt;
	Eigen::Vector3d un_gyr_normalized = Eigen::Vector3d::Zero();

	if (un_gyr_norm > 1e-12)
		un_gyr_normalized = un_gyr / un_gyr_norm;

	double dt_2 = std::pow(dt, 2);
	double dt_3 = std::pow(dt, 3);
	double norm_2 = std::pow(un_gyr_norm, 2);
	double norm_3 = std::pow(un_gyr_norm, 3);
	double cos_dth = std::cos(d_th);
	double sin_dth = std::sin(d_th);
	double d_th_2 = std::pow(d_th, 2);
	double d_th_3 = std::pow(d_th, 3);

	Eigen::Matrix3d sK = quatType::skewSymmetric(un_gyr_normalized);
	Eigen::Matrix3d sK2 = sK * sK;
	Eigen::Matrix3d sA = quatType::skewSymmetric(un_acc);

	Eigen::Matrix3d R_temp = quatType::expSo3(-un_gyr * dt);
	Eigen::Matrix3d Jr_temp = quatType::JrighySo3(-un_gyr * dt);

	Eigen::Matrix3d Xi_1, Xi_2, Xi_3, Xi_4;

  	if (!(un_gyr_norm < 1.0 / 180 * M_PI / 2))
  	{
		Xi_1 = Eigen::Matrix3d::Identity() * dt + (1.0 - cos_dth) / un_gyr_norm * sK + (dt - sin_dth / un_gyr_norm) * sK2;

		Xi_2 = 1.0 / 2 * dt_2 * Eigen::Matrix3d::Identity() + (d_th - sin_dth) / norm_2 * sK + (1.0 / 2 * dt_2 - (1.0 - cos_dth) / norm_2) * sK2;

		Xi_3 = 1.0 / 2 * dt_2 * sA + (sin_dth - d_th) / norm_2 * sA * sK + (sin_dth - d_th * cos_dth) / norm_2 * sK * sA +
			(1.0 / 2 * dt_2 - (1.0 - cos_dth) / norm_2) * sA * sK2 +
			(1.0 / 2 * dt_2 + (1.0 - cos_dth - d_th * sin_dth) / norm_2) * (sK2 * sA + un_gyr_normalized.dot(un_acc) * sK) -
			(3 * sin_dth - 2 * d_th - d_th * cos_dth) / norm_2 * un_gyr_normalized.dot(un_acc) * sK2;

		Xi_4 = 1.0 / 6 * dt_3 * sA + (2 * (1.0 - cos_dth) - d_th_2) / (2 * norm_3) * sA * sK +
			((2 * (1.0 - cos_dth) - d_th * sin_dth) / norm_3) * sK * sA + ((sin_dth - d_th) / norm_3 + dt_3 / 6) * sA * sK2 +
			((d_th - 2 * sin_dth + 1.0 / 6 * d_th_3 + d_th * cos_dth) / norm_3) * (sK2 * sA + un_gyr_normalized.dot(un_acc) * sK) +
			(4 * cos_dth - 4 + d_th_2 + d_th * sin_dth) / norm_3 * un_gyr_normalized.dot(un_acc) * sK2;

	}
	else
	{
		Xi_1 = dt * (Eigen::Matrix3d::Identity() + sin_dth * sK + (1.0 - cos_dth) * sK2);

		Xi_2 = 1.0 / 2 * dt * Xi_1;

		Xi_3 = 1.0 / 2 * dt_2 * (sA + sin_dth * (-sA * sK + sK * sA + un_gyr_normalized.dot(un_acc) * sK2) 
			+ (1.0 - cos_dth) * (sA * sK2 + sK2 * sA + un_gyr_normalized.dot(un_acc) * sK));

		Xi_4 = 1.0 / 3 * dt * Xi_3;
	}

	Xi_sum.setZero();
	Xi_sum.block<3, 3>(0, 0) = R_temp;
	Xi_sum.block<3, 3>(0, 3) = Xi_1;
	Xi_sum.block<3, 3>(0, 6) = Xi_2;
	Xi_sum.block<3, 3>(0, 9) = Jr_temp;
	Xi_sum.block<3, 3>(0, 12) = Xi_3;
	Xi_sum.block<3, 3>(0, 15) = Xi_4;
}

void propagator::predictMeanRk4(std::shared_ptr<state> state_ptr, double dt, const Eigen::Vector3d &un_gyr_0, const Eigen::Vector3d &un_acc_0,
	const Eigen::Vector3d &un_gyr_1, const Eigen::Vector3d &un_acc_1, Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p)
{
	Eigen::Vector3d un_gyr = un_gyr_0;
	Eigen::Vector3d un_acc = un_acc_0;
	Eigen::Vector3d w_alpha = (un_gyr_1 - un_gyr_0) / dt;
	Eigen::Vector3d a_jerk = (un_acc_1 - un_acc_0) / dt;

	Eigen::Vector4d q_0 = state_ptr->imu_ptr->getQuat();
	Eigen::Vector3d p_0 = state_ptr->imu_ptr->getPos();
	Eigen::Vector3d v_0 = state_ptr->imu_ptr->getVel();

	Eigen::Vector4d dq_0 = {0, 0, 0, 1};
	Eigen::Vector4d q_0_dot = 0.5 * quatType::omega(un_gyr) * dq_0;
	Eigen::Vector3d p_0_dot = v_0;
	Eigen::Matrix3d R_0_world = quatType::quatToRot(quatType::quatMultiply(dq_0, q_0));
	Eigen::Vector3d v_0_dot = R_0_world.transpose() * un_acc - gravity;

	Eigen::Vector4d k1_q = q_0_dot * dt;
	Eigen::Vector3d k1_p = p_0_dot * dt;
	Eigen::Vector3d k1_v = v_0_dot * dt;

	un_gyr += 0.5 * w_alpha * dt;
	un_acc += 0.5 * a_jerk * dt;

	Eigen::Vector4d dq_1 = quatType::quatNorm(dq_0 + 0.5 * k1_q);
	Eigen::Vector3d v_1 = v_0 + 0.5 * k1_v;
	Eigen::Vector4d q_1_dot = 0.5 * quatType::omega(un_gyr) * dq_1;
	Eigen::Vector3d p_1_dot = v_1;
	Eigen::Matrix3d R_1_world = quatType::quatToRot(quatType::quatMultiply(dq_1, q_0));
	Eigen::Vector3d v_1_dot = R_1_world.transpose() * un_acc - gravity;

	Eigen::Vector4d k2_q = q_1_dot * dt;
	Eigen::Vector3d k2_p = p_1_dot * dt;
	Eigen::Vector3d k2_v = v_1_dot * dt;

	Eigen::Vector4d dq_2 = quatType::quatNorm(dq_0 + 0.5 * k2_q);
	Eigen::Vector3d v_2 = v_0 + 0.5 * k2_v;

	Eigen::Vector4d q_2_dot = 0.5 * quatType::omega(un_gyr) * dq_2;
	Eigen::Vector3d p_2_dot = v_2;
	Eigen::Matrix3d R_2_world = quatType::quatToRot(quatType::quatMultiply(dq_2, q_0));
	Eigen::Vector3d v_2_dot = R_2_world.transpose() * un_acc - gravity;

	Eigen::Vector4d k3_q = q_2_dot * dt;
	Eigen::Vector3d k3_p = p_2_dot * dt;
	Eigen::Vector3d k3_v = v_2_dot * dt;

	un_gyr += 0.5 * w_alpha * dt;
	un_acc += 0.5 * a_jerk * dt;

	Eigen::Vector4d dq_3 = quatType::quatNorm(dq_0 + k3_q);
	Eigen::Vector3d v_3 = v_0 + k3_v;

	Eigen::Vector4d q_3_dot = 0.5 * quatType::omega(un_gyr) * dq_3;
	Eigen::Vector3d p_3_dot = v_3;
	Eigen::Matrix3d R_3_world = quatType::quatToRot(quatType::quatMultiply(dq_3, q_0));
	Eigen::Vector3d v_3_dot = R_3_world.transpose() * un_acc - gravity;

	Eigen::Vector4d k4_q = q_3_dot * dt;
	Eigen::Vector3d k4_p = p_3_dot * dt;
	Eigen::Vector3d k4_v = v_3_dot * dt;

	Eigen::Vector4d dq = quatType::quatNorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
	new_q = quatType::quatMultiply(dq, q_0);
	new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
	new_v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
}

void propagator::computeCoefficientAnalytic(std::shared_ptr<state> state_ptr, double dt, const Eigen::Vector3d &un_gyr, const Eigen::Vector3d &un_acc, 
	const Eigen::Vector3d &gyr_uncorrected, const Eigen::Vector3d &acc_uncorrected, const Eigen::Vector4d &new_q, const Eigen::Vector3d &new_v, 
	const Eigen::Vector3d &new_p, const Eigen::Matrix<double, 3, 18> &Xi_sum, Eigen::MatrixXd &F, Eigen::MatrixXd &G)
{
	int local_size = 0;
	int th_id = local_size;
	local_size += state_ptr->imu_ptr->q()->getSize();
	int p_id = local_size;
	local_size += state_ptr->imu_ptr->p()->getSize();
	int v_id = local_size;
	local_size += state_ptr->imu_ptr->v()->getSize();
	int bg_id = local_size;
	local_size += state_ptr->imu_ptr->bg()->getSize();
	int ba_id = local_size;
	local_size += state_ptr->imu_ptr->ba()->getSize();

	int Dw_id = -1;
	int Da_id = -1;
	int Tg_id = -1;
	int th_imu_acc_id = -1;
	int th_imu_gyr_id = -1;

	if (state_ptr->options.do_calib_imu_intrinsics)
	{
		Dw_id = local_size;
		local_size += state_ptr->calib_imu_dw->getSize();
		Da_id = local_size;
		local_size += state_ptr->calib_imu_da->getSize();

		if (state_ptr->options.do_calib_imu_g_sensitivity)
		{
			Tg_id = local_size;
			local_size += state_ptr->calib_imu_tg->getSize();
		}

		if (state_ptr->options.imu_model == ImuModel::KALIBR)
		{
			th_imu_gyr_id = local_size;
			local_size += state_ptr->calib_imu_gyr->getSize();
		}
		else
		{
			th_imu_acc_id = local_size;
			local_size += state_ptr->calib_imu_acc->getSize();
		}
	}

	Eigen::Matrix3d R_k = state_ptr->imu_ptr->getRot();
	Eigen::Vector3d v_k = state_ptr->imu_ptr->getVel();
	Eigen::Vector3d p_k = state_ptr->imu_ptr->getPos();

	if (state_ptr->options.do_fej)
	{
		R_k = state_ptr->imu_ptr->getRotFej();
		v_k = state_ptr->imu_ptr->getVelFej();
		p_k = state_ptr->imu_ptr->getPosFej();
	}

	Eigen::Matrix3d dR_temp = quatType::quatToRot(new_q) * R_k.transpose();

	Eigen::Matrix3d Dw = state::Dm(state_ptr->options.imu_model, state_ptr->calib_imu_dw->value());
	Eigen::Matrix3d Da = state::Dm(state_ptr->options.imu_model, state_ptr->calib_imu_da->value());
	Eigen::Matrix3d Tg = state::Tg(state_ptr->calib_imu_tg->value());
	Eigen::Matrix3d R_imu_acc = state_ptr->calib_imu_acc->getRot();
	Eigen::Matrix3d R_imu_gyr = state_ptr->calib_imu_gyr->getRot();
	Eigen::Vector3d acc_k = R_imu_acc * Da * acc_uncorrected;
	Eigen::Vector3d gyr_k = R_imu_gyr * Dw * gyr_uncorrected;

	Eigen::Matrix3d Xi_1 = Xi_sum.block<3, 3>(0, 3);
	Eigen::Matrix3d Xi_2 = Xi_sum.block<3, 3>(0, 6);
	Eigen::Matrix3d Jr_temp = Xi_sum.block<3, 3>(0, 9);
	Eigen::Matrix3d Xi_3 = Xi_sum.block<3, 3>(0, 12);
	Eigen::Matrix3d Xi_4 = Xi_sum.block<3, 3>(0, 15);

	F.block<3, 3>(th_id, th_id) = dR_temp;
	F.block<3, 3>(p_id, th_id) = - quatType::skewSymmetric(new_p - p_k - v_k * dt + 0.5 * gravity * dt * dt) * R_k.transpose();
	F.block<3, 3>(v_id, th_id) = - quatType::skewSymmetric(new_v - v_k + gravity * dt) * R_k.transpose();

	F.block<3, 3>(p_id, p_id).setIdentity();

	F.block<3, 3>(p_id, v_id) = Eigen::Matrix3d::Identity() * dt;
	F.block<3, 3>(v_id, v_id).setIdentity();

	F.block<3, 3>(th_id, bg_id) = - dR_temp * Jr_temp * dt * R_imu_gyr * Dw;
	F.block<3, 3>(p_id, bg_id) = R_k.transpose() * Xi_4 * R_imu_gyr * Dw;
	F.block<3, 3>(v_id, bg_id) = R_k.transpose() * Xi_3 * R_imu_gyr * Dw;
	F.block<3, 3>(bg_id, bg_id).setIdentity();

	F.block<3, 3>(th_id, ba_id) = dR_temp * Jr_temp * dt * R_imu_gyr * Dw * Tg * R_imu_acc * Da;
	F.block<3, 3>(p_id, ba_id) = - R_k.transpose() * (Xi_2 + Xi_4 * R_imu_gyr * Dw * Tg) * R_imu_acc * Da;
	F.block<3, 3>(v_id, ba_id) = - R_k.transpose() * (Xi_1 + Xi_3 * R_imu_gyr * Dw * Tg) * R_imu_acc * Da;
	F.block<3, 3>(ba_id, ba_id).setIdentity();

	if (Dw_id != -1)
	{
		Eigen::MatrixXd H_Dw = computeDwH(state_ptr, gyr_uncorrected);
		F.block(th_id, Dw_id, 3, state_ptr->calib_imu_dw->getSize()) = dR_temp * Jr_temp * dt * R_imu_gyr * H_Dw;
		F.block(p_id, Dw_id, 3, state_ptr->calib_imu_dw->getSize()) = - R_k.transpose() * Xi_4 * R_imu_gyr * H_Dw;
		F.block(v_id, Dw_id, 3, state_ptr->calib_imu_dw->getSize()) = - R_k.transpose() * Xi_3 * R_imu_gyr * H_Dw;
		F.block(Dw_id, Dw_id, state_ptr->calib_imu_dw->getSize(), state_ptr->calib_imu_dw->getSize()).setIdentity();
	}

	if (Da_id != -1)
	{
		Eigen::MatrixXd H_Da = computeDaH(state_ptr, acc_uncorrected);
		F.block(th_id, Da_id, 3, state_ptr->calib_imu_da->getSize()) = - dR_temp * Jr_temp * dt * R_imu_gyr * Dw * Tg * R_imu_acc * H_Da;
		F.block(p_id, Da_id, 3, state_ptr->calib_imu_da->getSize()) = R_k.transpose() * (Xi_2 + Xi_4 * R_imu_gyr * Dw * Tg) * R_imu_acc * H_Da;
		F.block(v_id, Da_id, 3, state_ptr->calib_imu_da->getSize()) = R_k.transpose() * (Xi_1 + Xi_3 * R_imu_gyr * Dw * Tg) * R_imu_acc * H_Da;
		F.block(Da_id, Da_id, state_ptr->calib_imu_da->getSize(), state_ptr->calib_imu_da->getSize()).setIdentity();
	}

	if (Tg_id != -1)
	{
		Eigen::MatrixXd H_Tg = computeTgH(state_ptr, acc_k);
		F.block(th_id, Tg_id, 3, state_ptr->calib_imu_tg->getSize()) = - dR_temp * Jr_temp * dt * R_imu_gyr * Dw * H_Tg;
		F.block(p_id, Tg_id, 3, state_ptr->calib_imu_tg->getSize()) = R_k.transpose() * Xi_4 * R_imu_gyr * Dw * H_Tg;
		F.block(v_id, Tg_id, 3, state_ptr->calib_imu_tg->getSize()) = R_k.transpose() * Xi_3 * R_imu_gyr * Dw * H_Tg;
		F.block(Tg_id, Tg_id, state_ptr->calib_imu_tg->getSize(), state_ptr->calib_imu_tg->getSize()).setIdentity();
	}

	if (th_imu_acc_id != -1)
	{
		F.block<3, 3>(th_id, th_imu_acc_id) = - dR_temp * Jr_temp * dt * R_imu_gyr * Dw * Tg * quatType::skewSymmetric(acc_k);
		F.block<3, 3>(p_id, th_imu_acc_id) = R_k.transpose() * (Xi_2 + Xi_4 * R_imu_gyr * Dw * Tg) * quatType::skewSymmetric(acc_k);
		F.block<3, 3>(v_id, th_imu_acc_id) = R_k.transpose() * (Xi_1 + Xi_3 * R_imu_gyr * Dw * Tg) * quatType::skewSymmetric(acc_k);
		F.block<3, 3>(th_imu_acc_id, th_imu_acc_id).setIdentity();
	}

	if (th_imu_gyr_id != -1)
	{
		F.block<3, 3>(th_id, th_imu_gyr_id) = dR_temp * Jr_temp * dt * quatType::skewSymmetric(gyr_k);
		F.block<3, 3>(p_id, th_imu_gyr_id) = -R_k.transpose() * Xi_4 * quatType::skewSymmetric(gyr_k);
		F.block<3, 3>(v_id, th_imu_gyr_id) = -R_k.transpose() * Xi_3 * quatType::skewSymmetric(gyr_k);
		F.block<3, 3>(th_imu_gyr_id, th_imu_gyr_id).setIdentity();
	}

	G.block<3, 3>(th_id, 0) = - dR_temp * Jr_temp * dt * R_imu_gyr * Dw;
	G.block<3, 3>(p_id, 0) = R_k.transpose() * Xi_4 * R_imu_gyr * Dw;
	G.block<3, 3>(v_id, 0) = R_k.transpose() * Xi_3 * R_imu_gyr * Dw;
	G.block<3, 3>(th_id, 3) = dR_temp * Jr_temp * dt * R_imu_gyr * Dw * Tg * R_imu_acc * Da;
	G.block<3, 3>(p_id, 3) = -R_k.transpose() * (Xi_2 + Xi_4 * R_imu_gyr * Dw * Tg) * R_imu_acc * Da;
	G.block<3, 3>(v_id, 3) = -R_k.transpose() * (Xi_1 + Xi_3 * R_imu_gyr * Dw * Tg) * R_imu_acc * Da;
	G.block<3, 3>(bg_id, 6) = dt * Eigen::Matrix3d::Identity();
	G.block<3, 3>(ba_id, 9) = dt * Eigen::Matrix3d::Identity();
}

Eigen::MatrixXd propagator::computeDwH(std::shared_ptr<state> state_ptr, const Eigen::Vector3d &gyr_uncorrected)
{
	Eigen::Vector3d e_1 = Eigen::MatrixXd::Identity(3, 3).block<3, 1>(0, 0);
	Eigen::Vector3d e_2 = Eigen::MatrixXd::Identity(3, 3).block<3, 1>(0, 1);
	Eigen::Vector3d e_3 = Eigen::MatrixXd::Identity(3, 3).block<3, 1>(0, 2);

	double w_1 = gyr_uncorrected(0);
	double w_2 = gyr_uncorrected(1);
	double w_3 = gyr_uncorrected(2);
	assert(state_ptr->options.do_calib_imu_intrinsics);

	Eigen::MatrixXd H_Dw = Eigen::MatrixXd::Zero(3, 6);
	if (state_ptr->options.imu_model == ImuModel::KALIBR)
		H_Dw << w_1 * Eigen::MatrixXd::Identity(3, 3), w_2 * e_2, w_2 * e_3, w_3 * e_3;
	else
		H_Dw << w_1 * e_1, w_2 * e_1, w_2 * e_2, w_3 * Eigen::MatrixXd::Identity(3, 3);

	return H_Dw;
}

Eigen::MatrixXd propagator::computeDaH(std::shared_ptr<state> state_ptr, const Eigen::Vector3d &acc_uncorrected)
{
	Eigen::Vector3d e_1 = Eigen::MatrixXd::Identity(3, 3).block<3, 1>(0, 0);
	Eigen::Vector3d e_2 = Eigen::MatrixXd::Identity(3, 3).block<3, 1>(0, 1);
	Eigen::Vector3d e_3 = Eigen::MatrixXd::Identity(3, 3).block<3, 1>(0, 2);

	double a_1 = acc_uncorrected(0);
	double a_2 = acc_uncorrected(1);
	double a_3 = acc_uncorrected(2);
	assert(state_ptr->options.do_calib_imu_intrinsics);

	Eigen::MatrixXd H_Da = Eigen::MatrixXd::Zero(3, 6);
	if (state_ptr->options.imu_model == ImuModel::KALIBR)
		H_Da << a_1 * Eigen::MatrixXd::Identity(3, 3), a_2 * e_2, a_2 * e_3, a_3 * e_3;
	else
		H_Da << a_1 * e_1, a_2 * e_1, a_2 * e_2, a_3 * Eigen::MatrixXd::Identity(3, 3);

	return H_Da;
}

Eigen::MatrixXd propagator::computeTgH(std::shared_ptr<state> state_ptr, const Eigen::Vector3d &acc)
{
	double a_1 = acc(0);
	double a_2 = acc(1);
	double a_3 = acc(2);
	assert(state_ptr->options.do_calib_imu_intrinsics);
	assert(state_ptr->options.do_calib_imu_g_sensitivity);

	Eigen::MatrixXd H_Tg = Eigen::MatrixXd::Zero(3, 9);
	H_Tg << a_1 * Eigen::MatrixXd::Identity(3, 3), a_2 * Eigen::MatrixXd::Identity(3, 3), a_3 * Eigen::MatrixXd::Identity(3, 3);

	return H_Tg;
}



updaterMsckf::updaterMsckf(updaterOptions &options_, featureInitializerOptions &feature_init_options_) : options(options_)
{
	options.sigma_pix_sq = std::pow(options.sigma_pix, 2);

	initializer_feature = std::shared_ptr<featureInitializer>(new featureInitializer(feature_init_options_));

	for (int i = 1; i < 500; i++)
	{
		boost::math::chi_squared chi_squared_dist(i);
		chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
	}
}

void updaterMsckf::update(std::shared_ptr<state> state_ptr, std::vector<std::shared_ptr<feature>> &feature_vec)
{
	if (feature_vec.empty())
		return;

	boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
	rT0 = boost::posix_time::microsec_clock::local_time();

	std::vector<double> clone_times;
	for (const auto &clone_imu : state_ptr->clones_imu)
	{
		clone_times.emplace_back(clone_imu.first);
	}

	auto it0 = feature_vec.begin();
	while (it0 != feature_vec.end())
	{
		(*it0)->cleanOldMeasurements(clone_times);

		int count_meas = 0;
		for (const auto &pair : (*it0)->timestamps)
		{
			count_meas += (*it0)->timestamps[pair.first].size();
		}

		if (count_meas < 2)
		{
			(*it0)->to_delete = true;
			it0 = feature_vec.erase(it0);
		}
		else
		{
			it0++;
		}
	}
	rT1 = boost::posix_time::microsec_clock::local_time();

	std::unordered_map<size_t, std::unordered_map<double, featureInitializer::clonePose>> clones_cam;
	for (const auto &clone_calib : state_ptr->calib_cam_imu)
	{
		std::unordered_map<double, featureInitializer::clonePose> clones_cami;
		for (const auto &clone_imu : state_ptr->clones_imu)
		{
			Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->getRot() * clone_imu.second->getRot();
			Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->getPos() - R_GtoCi.transpose() * clone_calib.second->getPos();

			clones_cami.insert({clone_imu.first, featureInitializer::clonePose(R_GtoCi, p_CioinG)});
		}

		clones_cam.insert({clone_calib.first, clones_cami});
	}

	auto it1 = feature_vec.begin();
	while (it1 != feature_vec.end())
	{
		bool success_tri = true;
		success_tri = initializer_feature->singleTriangulation(*it1, clones_cam);

		bool success_refine = true;
		if (initializer_feature->config().refine_features)
		{
			success_refine = initializer_feature->singleGaussnewton(*it1, clones_cam);
		}

		if (!success_tri || !success_refine)
		{
			(*it1)->to_delete = true;
			it1 = feature_vec.erase(it1);
			continue;
		}
		
		it1++;
	}
	rT2 = boost::posix_time::microsec_clock::local_time();

	size_t max_meas_size = 0;
	for (size_t i = 0; i < feature_vec.size(); i++)
	{
		for (const auto &pair : feature_vec.at(i)->timestamps)
		{
			max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
		}
	}

	size_t max_hx_size = state_ptr->maxCovSize();
	for (auto &map_point : state_ptr->map_points)
	{
		max_hx_size -= map_point.second->getSize();
	}

	Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
	Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
	std::unordered_map<std::shared_ptr<baseType>, size_t> Hx_mapping;
	std::vector<std::shared_ptr<baseType>> Hx_order_big;
	size_t count_jacob = 0;
	size_t count_meas = 0;

	auto it2 = feature_vec.begin();
	while (it2 != feature_vec.end())
	{
		updaterHelper::updaterHelperFeature feat;
		feat.feature_id = (*it2)->feature_id;
		feat.uvs = (*it2)->uvs;
		feat.uvs_norm = (*it2)->uvs_norm;
		feat.timestamps = (*it2)->timestamps;
		feat.frames = (*it2)->frames;
		feat.color = (*it2)->color;
		feat.position_global = (*it2)->position_global;
		feat.position_global_fej = (*it2)->position_global;

		Eigen::MatrixXd H_f;
		Eigen::MatrixXd H_x;
		Eigen::VectorXd res;
		std::vector<std::shared_ptr<baseType>> Hx_order;

		updaterHelper::getFeatureJacobianFull(state_ptr, feat, H_f, H_x, res, Hx_order);
		updaterHelper::nullspaceProjectInplace(H_f, H_x, res);

		Eigen::MatrixXd P_marg = stateHelper::getMarginalCovariance(state_ptr, Hx_order);
		Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
		S.diagonal() += options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
		double chi2 = res.dot(S.llt().solve(res));

		double chi2_check;
		if (res.rows() < 500)
		{
			chi2_check = chi_squared_table[res.rows()];
		}
		else
		{
			boost::math::chi_squared chi_squared_dist(res.rows());
			chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
			std::cout << "chi2_check over the residual limit - " << (int)res.rows() << std::endl;
		}

		if (chi2 > options.chi2_multipler * chi2_check)
		{
			(*it2)->to_delete = true;
			it2 = feature_vec.erase(it2);

			continue;
		}

		size_t count_hx = 0;
		for (const auto &var : Hx_order)
		{
			if (Hx_mapping.find(var) == Hx_mapping.end())
			{
				Hx_mapping.insert({var, count_jacob});
				Hx_order_big.push_back(var);
				count_jacob += var->getSize();
			}

			Hx_big.block(count_meas, Hx_mapping[var], H_x.rows(), var->getSize()) = H_x.block(0, count_hx, H_x.rows(), var->getSize());
			count_hx += var->getSize();
		}

		res_big.block(count_meas, 0, res.rows(), 1) = res;
		count_meas += res.rows();
		it2++;
	}
	rT3 = boost::posix_time::microsec_clock::local_time();

	for (size_t f = 0; f < feature_vec.size(); f++)
	{
		if ((feature_vec[f]->timestamps.find(0) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(0).empty() && feature_vec[f]->timestamps.at(0).back() > clone_times.at(clone_times.size() - 2) + 1e-5) || 
			(feature_vec[f]->timestamps.find(1) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(1).empty() && feature_vec[f]->timestamps.at(1).back() > clone_times.at(clone_times.size() - 2) + 1e-5))
		{
			if ((feature_vec[f]->timestamps.find(0) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(0).size() && feature_vec[f]->timestamps.at(0).front() < clone_times.at(clone_times.size() - 3) - 1e-5) || 
			(feature_vec[f]->timestamps.find(1) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(1).empty() && feature_vec[f]->timestamps.at(1).front() < clone_times.at(clone_times.size() - 3) - 1e-5))
				feature_vec[f]->to_delete = true;
		}
		else
		{
			feature_vec[f]->to_delete = true;
		}
	}

	if (count_meas < 1)
	{
		return;
	}

	assert(count_meas <= max_meas_size);
	assert(count_jacob <= max_hx_size);
  
	res_big.conservativeResize(count_meas, 1);
	Hx_big.conservativeResize(count_meas, count_jacob);

	updaterHelper::measurementCompressInplace(Hx_big, res_big);
	if (Hx_big.rows() < 1)
	{
		return;
	}
	rT4 = boost::posix_time::microsec_clock::local_time();

	Eigen::MatrixXd R_big = options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());

	stateHelper::ekfUpdate(state_ptr, Hx_order_big, Hx_big, res_big, R_big);
	rT5 = boost::posix_time::microsec_clock::local_time();

	// Time test
	/*
	std::cout << std::fixed << "[updaterMsckf]: " << (rT1 - rT0).total_microseconds() * 1e-6 << " seconds to clean." << std::endl;
	std::cout << std::fixed << "[updaterMsckf]: " << (rT2 - rT1).total_microseconds() * 1e-6 << " seconds to triangulate." << std::endl;
	std::cout << std::fixed << "[updaterMsckf]: " << (rT3 - rT2).total_microseconds() * 1e-6 << " seconds create system (" << (int)feature_vec.size() << " features)." << std::endl;
	std::cout << std::fixed << "[updaterMsckf]: " << (rT4 - rT3).total_microseconds() * 1e-6 << " seconds compress system." << std::endl;
	std::cout << std::fixed << "[updaterMsckf]: " << (rT5 - rT4).total_microseconds() * 1e-6 << " seconds update state (" << (int)res_big.rows() << " size)." << std::endl;
	std::cout << std::fixed << "[updaterMsckf]: " << (rT5 - rT1).total_microseconds() * 1e-6 << " seconds total." << std::endl;
	*/
	// Time test
}



updaterSlam::updaterSlam(updaterOptions &options_slam_, featureInitializerOptions &feat_init_options_, std::shared_ptr<featureDatabase> database_) 
	: options_slam(options_slam_), database(database_)
{
	options_slam.sigma_pix_sq = std::pow(options_slam.sigma_pix, 2);

	initializer_feature = std::shared_ptr<featureInitializer>(new featureInitializer(feat_init_options_));

	for (int i = 1; i < 500; i++)
	{
		boost::math::chi_squared chi_squared_dist(i);
		chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
	}
}

void updaterSlam::delayedInit(std::shared_ptr<state> state_ptr, voxelHashMap &voxel_map, std::vector<std::shared_ptr<feature>> &feature_vec, pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history)
{
	if (feature_vec.empty())
		return;

	boost::posix_time::ptime rT0, rT1, rT2, rT3;
	rT0 = boost::posix_time::microsec_clock::local_time();

	std::vector<double> clone_times;
	for (const auto &clone_imu : state_ptr->clones_imu)
	{
		clone_times.emplace_back(clone_imu.first);
	}

	auto it0 = feature_vec.begin();
	while (it0 != feature_vec.end())
	{
		(*it0)->cleanOldMeasurements(clone_times);

		int count_meas = 0;
		for (const auto &pair : (*it0)->timestamps)
		{
			count_meas += (*it0)->timestamps[pair.first].size();
		}

		if (count_meas < 2)
		{
			(*it0)->to_delete = true;
			it0 = feature_vec.erase(it0);
		}
		else
		{
			it0++;
		}
	}
	rT1 = boost::posix_time::microsec_clock::local_time();

	std::unordered_map<size_t, std::unordered_map<double, featureInitializer::clonePose>> clones_cam;
	for (const auto &clone_calib : state_ptr->calib_cam_imu)
	{
		std::unordered_map<double, featureInitializer::clonePose> clones_cami;
		for (const auto &clone_imu : state_ptr->clones_imu)
		{
			Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->getRot() * clone_imu.second->getRot();
			Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->getPos() - R_GtoCi.transpose() * clone_calib.second->getPos();

			clones_cami.insert({clone_imu.first, featureInitializer::clonePose(R_GtoCi, p_CioinG)});
		}

		clones_cam.insert({clone_calib.first, clones_cami});
	}

	auto it1 = feature_vec.begin();
	while (it1 != feature_vec.end())
	{
		bool success_tri = true;
		success_tri = initializer_feature->singleTriangulation(*it1, clones_cam);

		bool success_refine = true;
		if (initializer_feature->config().refine_features)
		{
			success_refine = initializer_feature->singleGaussnewton(*it1, clones_cam);
		}

		if (!success_tri || !success_refine)
		{
			(*it1)->to_delete = true;
			it1 = feature_vec.erase(it1);
			continue;
		}
		it1++;
	}
	rT2 = boost::posix_time::microsec_clock::local_time();

	auto it2 = feature_vec.begin();
	while (it2 != feature_vec.end())
	{	
		updaterHelper::updaterHelperFeature feat;
		feat.feature_id = (*it2)->feature_id;
		feat.uvs = (*it2)->uvs;
		feat.uvs_norm = (*it2)->uvs_norm;
		feat.timestamps = (*it2)->timestamps;

		feat.frames = (*it2)->frames;
		feat.color = (*it2)->color;
		assert(feat.color[0].size() == feat.uvs[0].size());
		assert(feat.color[1].size() == feat.uvs[1].size());
		assert(feat.color[(*it2)->anchor_cam_id].back() >= 0);

		feat.anchor_cam_id = (*it2)->anchor_cam_id;
		feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;

		feat.position_global = (*it2)->position_global;
		feat.position_global_fej = (*it2)->position_global;

		Eigen::MatrixXd H_f;
		Eigen::MatrixXd H_x;
		Eigen::VectorXd res;
		std::vector<std::shared_ptr<baseType>> Hx_order;

		updaterHelper::getFeatureJacobianFull(state_ptr, feat, H_f, H_x, res, Hx_order);

		auto map_point_ptr = std::make_shared<mapPoint>();
		map_point_ptr->feature_id = feat.feature_id;
		map_point_ptr->unique_camera_id = (*it2)->anchor_cam_id;

		map_point_ptr->color = feat.color[feat.anchor_cam_id].back();
		map_point_ptr->host_frame = feat.frames[(*it2)->anchor_cam_id].back();
		map_point_ptr->anchor_cam_id = feat.anchor_cam_id;
		map_point_ptr->anchor_clone_timestamp = feat.anchor_clone_timestamp;

		map_point_ptr->setPointXYZ(feat.position_global, false);
		map_point_ptr->setPointXYZ(feat.position_global_fej, true);

		double sigma_pix_sq = options_slam.sigma_pix_sq;
		Eigen::MatrixXd R = sigma_pix_sq * Eigen::MatrixXd::Identity(res.rows(), res.rows());

		double chi2_multipler = options_slam.chi2_multipler;

		if (stateHelper::initialize(state_ptr, map_point_ptr, Hx_order, H_x, H_f, R, res, chi2_multipler))
		{
			bool add = mapManagement::addPointToVoxel(voxel_map, map_point_ptr, state_ptr->options.voxel_size, state_ptr->options.max_num_points_in_voxel, 
				state_ptr->options.min_distance_points, voxels_history);

			if (add)
			{
				state_ptr->map_points.insert({(*it2)->feature_id, map_point_ptr});
				(*it2)->to_delete = true;
				it2++;
			}
			else
			{
				(*it2)->to_delete = true;
				it2 = feature_vec.erase(it2);
			}	
		}
		else
		{
			(*it2)->to_delete = true;
			it2 = feature_vec.erase(it2);
		}
	}
	rT3 = boost::posix_time::microsec_clock::local_time();

	// Time test
	/*
	if (!feature_vec.empty())
	{
		std::cout << std::fixed << "[updaterSlam::delayedInit]: " << (rT1 - rT0).total_microseconds() * 1e-6 << " seconds to clean." << std::endl;
		std::cout << std::fixed << "[updaterSlam::delayedInit]: " << (rT2 - rT1).total_microseconds() * 1e-6 << " seconds to triangulate." << std::endl;
		std::cout << std::fixed << "[updaterSlam::delayedInit]: " << (rT3 - rT2).total_microseconds() * 1e-6 << " seconds initialize (" << (int)feature_vec.size() << " features)." << std::endl;
		std::cout << std::fixed << "[updaterSlam::delayedInit]: " << (rT3 - rT1).total_microseconds() * 1e-6 << " seconds total." << std::endl;
	}
	*/
	// Time test
}

void updaterSlam::update(std::shared_ptr<state> state_ptr, voxelHashMap &voxel_map, std::vector<std::shared_ptr<feature>> &feature_vec)
{
	if (feature_vec.empty())
		return;

	boost::posix_time::ptime rT0, rT1, rT2, rT3;
	rT0 = boost::posix_time::microsec_clock::local_time();

	std::vector<double> clone_times;
	for (const auto &clone_imu : state_ptr->clones_imu)
	{
		clone_times.emplace_back(clone_imu.first);
	}

	auto it0 = feature_vec.begin();
	while (it0 != feature_vec.end())
	{
		(*it0)->cleanOldMeasurements(clone_times);

		int count_meas = 0;
		for (const auto &pair : (*it0)->timestamps)
		{
			count_meas += (*it0)->timestamps[pair.first].size();
		}

		std::shared_ptr<mapPoint> map_point = state_ptr->map_points.at((*it0)->feature_id);
		int required_meas = 2;

		if (count_meas < 1)
		{
			(*it0)->to_delete = true;
			it0 = feature_vec.erase(it0);
		}
		else if (count_meas < required_meas)
		{
			it0 = feature_vec.erase(it0);
		}
		else
		{
			it0++;
		}
	}
	rT1 = boost::posix_time::microsec_clock::local_time();

	size_t max_meas_size = 0;
	for (size_t i = 0; i < feature_vec.size(); i++)
	{
		for (const auto &pair : feature_vec.at(i)->timestamps)
		{
			max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
		}
	}

	size_t max_hx_size = state_ptr->maxCovSize();

	Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
	Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
	Eigen::MatrixXd R_big = Eigen::MatrixXd::Identity(max_meas_size, max_meas_size);
	std::unordered_map<std::shared_ptr<baseType>, size_t> Hx_mapping;
	std::vector<std::shared_ptr<baseType>> Hx_order_big;
	size_t count_jacob = 0;
	size_t count_meas = 0;

	auto it2 = feature_vec.begin();
	while (it2 != feature_vec.end())
	{
		assert(state_ptr->map_points.find((*it2)->feature_id) != state_ptr->map_points.end());
		assert(state_ptr->map_points.at((*it2)->feature_id)->feature_id == (*it2)->feature_id);

		std::shared_ptr<mapPoint> map_point = state_ptr->map_points.at((*it2)->feature_id);

		updaterHelper::updaterHelperFeature feat;
		feat.feature_id = (*it2)->feature_id;
		feat.uvs = (*it2)->uvs;
		feat.uvs_norm = (*it2)->uvs_norm;
		feat.timestamps = (*it2)->timestamps;
		feat.frames = (*it2)->frames;
		feat.color = (*it2)->color;

		feat.position_global = map_point->getPointXYZ(false);
		feat.position_global_fej = map_point->getPointXYZ(true);

		Eigen::MatrixXd H_f;
		Eigen::MatrixXd H_x;
		Eigen::VectorXd res;
		std::vector<std::shared_ptr<baseType>> Hx_order;

		updaterHelper::getFeatureJacobianFull(state_ptr, feat, H_f, H_x, res, Hx_order);

		Eigen::MatrixXd H_xf = H_x;

		H_xf.conservativeResize(H_x.rows(), H_x.cols() + H_f.cols());
		H_xf.block(0, H_x.cols(), H_x.rows(), H_f.cols()) = H_f;

		std::vector<std::shared_ptr<baseType>> Hxf_order = Hx_order;
		Hxf_order.push_back(map_point);

		Eigen::MatrixXd P_marg = stateHelper::getMarginalCovariance(state_ptr, Hxf_order);
		Eigen::MatrixXd S = H_xf * P_marg * H_xf.transpose();
		double sigma_pix_sq = options_slam.sigma_pix_sq;
		S.diagonal() += sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
		double chi2 = res.dot(S.llt().solve(res));

		double chi2_check;
		if (res.rows() < 500)
		{
			chi2_check = chi_squared_table[res.rows()];
		}
		else
		{
			boost::math::chi_squared chi_squared_dist(res.rows());
			chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
			std::cout << "chi2_check over the residual limit - " << (int)res.rows() << std::endl;
		}

		double chi2_multipler = options_slam.chi2_multipler;
		
		if (chi2 > chi2_multipler * chi2_check)
		{
			map_point->update_fail_count++;

			(*it2)->to_delete = true;
			it2 = feature_vec.erase(it2);
			continue;
		}

		size_t count_hx = 0;
		for (const auto &var : Hxf_order)
		{
			if (Hx_mapping.find(var) == Hx_mapping.end())
			{
				Hx_mapping.insert({var, count_jacob});
				Hx_order_big.push_back(var);
				count_jacob += var->getSize();
			}

			Hx_big.block(count_meas, Hx_mapping[var], H_xf.rows(), var->getSize()) = H_xf.block(0, count_hx, H_xf.rows(), var->getSize());
			count_hx += var->getSize();
		}

		R_big.block(count_meas, count_meas, res.rows(), res.rows()) *= sigma_pix_sq;

		res_big.block(count_meas, 0, res.rows(), 1) = res;
		count_meas += res.rows();
		it2++;
	}
	rT2 = boost::posix_time::microsec_clock::local_time();

	for (size_t f = 0; f < feature_vec.size(); f++)
	{
		if ((feature_vec[f]->timestamps.find(0) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(0).empty() && feature_vec[f]->timestamps.at(0).back() > clone_times.at(clone_times.size() - 1) + 1e-5) || 
			(feature_vec[f]->timestamps.find(1) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(1).empty() && feature_vec[f]->timestamps.at(1).back() > clone_times.at(clone_times.size() - 1) + 1e-5))
		{
			if ((feature_vec[f]->timestamps.find(0) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(0).empty() && feature_vec[f]->timestamps.at(0).front() < clone_times.at(clone_times.size() - 3) - 1e-5) || 
			(feature_vec[f]->timestamps.find(1) != feature_vec[f]->timestamps.end() && !feature_vec[f]->timestamps.at(1).empty() && feature_vec[f]->timestamps.at(1).front() < clone_times.at(clone_times.size() - 3) - 1e-5))
				feature_vec[f]->to_delete = true;
		}
		else
		{
			feature_vec[f]->to_delete = true;
		}
	}

	if (count_meas < 1)
	{
		return;
	}
	assert(count_meas <= max_meas_size);
	assert(count_jacob <= max_hx_size);
	res_big.conservativeResize(count_meas, 1);
	Hx_big.conservativeResize(count_meas, count_jacob);
	R_big.conservativeResize(count_meas, count_meas);

	stateHelper::ekfUpdate(state_ptr, Hx_order_big, Hx_big, res_big, R_big);
	rT3 = boost::posix_time::microsec_clock::local_time();

	// Time test
	/*
	std::cout << std::fixed << "[updaterSlam]: " << (rT1 - rT0).total_microseconds() * 1e-6 << " seconds to clean." << std::endl;
	std::cout << std::fixed << "[updaterSlam]: " << (rT2 - rT1).total_microseconds() * 1e-6 << " seconds creating linear system." << std::endl;
	std::cout << std::fixed << "[updaterSlam]: " << (rT3 - rT2).total_microseconds() * 1e-6 << " seconds to update (" 
		<< (int)feature_vec.size() << " feats of " << (int)Hx_big.rows() << " size)." << std::endl;
	std::cout << std::fixed << "[updaterSlam]: " << (rT3 - rT1).total_microseconds() * 1e-6 << " seconds total." << std::endl;
	*/
	// Time test
}