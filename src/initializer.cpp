#include "initializer.h"
#include "initializerHelper.h"
#include "featureHelper.h"

inertialInitializer::inertialInitializer(inertialInitializerOptions &initializer_options_,
	std::shared_ptr<featureDatabase> db_) : initializer_options(initializer_options_), db(db_)
{
	imu_data = std::make_shared<std::vector<imuData>>();

	static_initializer = std::make_shared<staticInitializer>(initializer_options, db, imu_data);
	dynamic_initializer = std::make_shared<dynamicInitializer>(initializer_options, db, imu_data);
}

void inertialInitializer::feedImu(const imuData &imu_data_, double oldest_time)
{
	imu_data->emplace_back(imu_data_);

	if (oldest_time != -1)
	{
		auto it0 = imu_data->begin();
    
		while (it0 != imu_data->end())
		{
			if (it0->timestamp < oldest_time)
				it0 = imu_data->erase(it0);
			else
				it0++;
		}
	}
}

bool inertialInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<baseType>> &order,
	std::shared_ptr<imuState> imu_state_, bool wait_for_jerk)
{
	double newest_cam_time = -1;

	for (auto const &feat : db->getInternalData())
	{
		for (auto const &cam_time_pair : feat.second->timestamps)
		{
			for (auto const &time : cam_time_pair.second)
			{
				newest_cam_time = std::max(newest_cam_time, time);
			}
		}
	}

	double oldest_time = newest_cam_time - initializer_options.init_window_time - 0.10;

	if (newest_cam_time < 0 || oldest_time < 0) return false;

	db->cleanUpOldmeasurements(oldest_time);
	auto it_imu = imu_data->begin();

	while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + initializer_options.calib_camimu_dt)
		it_imu = imu_data->erase(it_imu);

	bool disparity_detected_moving_1_to_0 = false;
	bool disparity_detected_moving_2_to_1 = false;
  
	if (initializer_options.init_max_disparity > 0)
	{
		double newest_time_allowed = newest_cam_time - 0.5 * initializer_options.init_window_time;
		int num_features_0 = 0;
		int num_features_1 = 0;
		double avg_disp_0, avg_disp_1;
		double var_disp_0, var_disp_1;
		featureHelper::computeDisparity(db, avg_disp_0, var_disp_0, num_features_0, newest_time_allowed);
		featureHelper::computeDisparity(db, avg_disp_1, var_disp_1, num_features_1, newest_cam_time, newest_time_allowed);

		int feat_thresh = 15;
		if (num_features_0 < feat_thresh || num_features_1 < feat_thresh)
		{
			std::cout << "[initialize]: Not enough feats to compute disp: " << num_features_0 << ","
				<< num_features_1 << " < " << feat_thresh << std::endl;
			return false;
		}

		std::cout << std::fixed << "[initialize]: Disparity is " << avg_disp_0 << "," 
			<< avg_disp_1 << "(" << initializer_options.init_max_disparity << " thresh)" << std::endl;

		disparity_detected_moving_1_to_0 = (avg_disp_0 > initializer_options.init_max_disparity);
		disparity_detected_moving_2_to_1 = (avg_disp_1 > initializer_options.init_max_disparity);
	}

	bool has_jerk = (!disparity_detected_moving_1_to_0 && disparity_detected_moving_2_to_1);
	bool is_still = (!disparity_detected_moving_1_to_0 && !disparity_detected_moving_2_to_1);

	if (((has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk)) && initializer_options.init_imu_thresh > 0.0)
	{
		std::cout << "[initialize]: Using static initializer method" << std::endl;
		return static_initializer->initialize(timestamp, covariance, order, imu_state_, wait_for_jerk);
	}
	else if (initializer_options.init_dyn_use && !is_still)
	{
		std::cout << "[initialize]: Using dynamic initializer method" << std::endl;
		std::map<double, std::shared_ptr<poseJpl>> clones_imu;
		std::unordered_map<size_t, std::shared_ptr<mapPoint>> map_points;
		return dynamic_initializer->initialize(timestamp, covariance, order, imu_state_, clones_imu, map_points);
	}
	else
	{
		std::string msg = (has_jerk) ? "" : "no accel jerk detected";
		msg += (has_jerk || is_still) ? "" : ", ";
		msg += (is_still) ? "" : "platform moving too much";
		std::cout << "[initialize]: Failed static init: " << msg << std::endl;
	}

	return false;
}



staticInitializer::staticInitializer(inertialInitializerOptions &initializer_options_, std::shared_ptr<featureDatabase> db_,
    std::shared_ptr<std::vector<imuData>> imu_data_) : initializer_options(initializer_options_), db(db_), imu_data(imu_data_)
{

}

bool staticInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<baseType>> &order, 
	std::shared_ptr<imuState> imu_state_, bool wait_for_jerk)
{
	if (imu_data->size() < 2) return false;

	double newest_time = imu_data->at(imu_data->size() - 1).timestamp;
	double oldest_time = imu_data->at(0).timestamp;

	if (newest_time - oldest_time < initializer_options.init_window_time)
	{
		std::cout << "[initialize-static]: Not enough IMU measurements" << std::endl;
		return false;
	}

	std::vector<imuData> window_1_to_0, window_2_to_1;
	for (const imuData &data : *imu_data)
	{
		if (data.timestamp > newest_time - 0.5 * initializer_options.init_window_time 
			&& data.timestamp <= newest_time - 0.0 * initializer_options.init_window_time)
			window_1_to_0.push_back(data);

		if (data.timestamp > newest_time - 1.0 * initializer_options.init_window_time 
			&& data.timestamp <= newest_time - 0.5 * initializer_options.init_window_time)
			window_2_to_1.push_back(data);
	}

	if (window_1_to_0.size() < 2 || window_2_to_1.size() < 2)
	{
		std::cout << "[initialize-static]: The frequency of IMU measurements is too low" << std::endl;
		return false;
	}

	Eigen::Vector3d a_avg_1_to_0 = Eigen::Vector3d::Zero();

	for (const imuData &data : window_1_to_0)
		a_avg_1_to_0 += data.acc;

	a_avg_1_to_0 /= (int)window_1_to_0.size();

	double a_var_1_to_0 = 0;
  
	for (const imuData &data : window_1_to_0)
		a_var_1_to_0 += (data.acc - a_avg_1_to_0).dot(data.acc - a_avg_1_to_0);

	a_var_1_to_0 = std::sqrt(a_var_1_to_0 / ((int)window_1_to_0.size() - 1));

	Eigen::Vector3d a_avg_2_to_1 = Eigen::Vector3d::Zero();
	Eigen::Vector3d w_avg_2_to_1 = Eigen::Vector3d::Zero();
	for (const imuData &data : window_2_to_1)
	{
		a_avg_2_to_1 += data.acc;
		w_avg_2_to_1 += data.gyr;
	}
	a_avg_2_to_1 = a_avg_2_to_1 / window_2_to_1.size();
	w_avg_2_to_1 = w_avg_2_to_1 / window_2_to_1.size();
  
  	double a_var_2_to_1 = 0;
	for (const imuData &data : window_2_to_1)
		a_var_2_to_1 += (data.acc - a_avg_2_to_1).dot(data.acc - a_avg_2_to_1);

	a_var_2_to_1 = std::sqrt(a_var_2_to_1 / ((int)window_2_to_1.size() - 1));
	std::cout << "[initialize-static]: IMU excitation stats: " << a_var_2_to_1 << ", " << a_var_1_to_0 << std::endl;

	if (a_var_1_to_0 < initializer_options.init_imu_thresh && wait_for_jerk)
	{
		std::cout << "[initialize-static]: No IMU excitation, below threshold " << a_var_1_to_0 << " < " 
			<< initializer_options.init_imu_thresh << std::endl;
		return false;
	}

	if (a_var_2_to_1 > initializer_options.init_imu_thresh && wait_for_jerk)
	{
		std::cout << "[initialize-static]: Too much IMU excitation, above threshold " << a_var_2_to_1 << " > " 
			<< initializer_options.init_imu_thresh << std::endl;
		return false;
	}

	if ((a_var_1_to_0 > initializer_options.init_imu_thresh || a_var_2_to_1 > initializer_options.init_imu_thresh) && !wait_for_jerk)
	{
		std::cout << "[initialize-static]: Tooo much IMU excitation, above threshold " << a_var_2_to_1 << ", " 
			<< a_var_1_to_0 << " > " << initializer_options.init_imu_thresh << std::endl;
		return false;
	}

	Eigen::Vector3d z_axis = a_avg_2_to_1 / a_avg_2_to_1.norm();
	Eigen::Matrix3d Ro;
	initializerHelper::gramSchmidt(z_axis, Ro);
	Eigen::Vector4d q_GtoI = quatType::rotToQuat(Ro);

	Eigen::Vector3d gravity_inG;
	gravity_inG << 0.0, 0.0, initializer_options.gravity_mag;
	Eigen::Vector3d bg = w_avg_2_to_1;
	Eigen::Vector3d ba = a_avg_2_to_1 - quatType::quatToRot(q_GtoI) * gravity_inG;

	timestamp = window_2_to_1.at(window_2_to_1.size() - 1).timestamp;
	Eigen::VectorXd imu_state = Eigen::VectorXd::Zero(16);
	imu_state.block(0, 0, 4, 1) = q_GtoI;
	imu_state.block(10, 0, 3, 1) = bg;
	imu_state.block(13, 0, 3, 1) = ba;
	assert(imu_state_ != nullptr);
	imu_state_->setValue(imu_state);
	imu_state_->setFej(imu_state);

	order.clear();
	order.push_back(imu_state_);
	covariance = std::pow(0.02, 2) * Eigen::MatrixXd::Identity(imu_state_->getSize(), imu_state_->getSize());
	covariance.block(0, 0, 3, 3) = std::pow(0.02, 2) * Eigen::Matrix3d::Identity(); // q
	covariance.block(3, 3, 3, 3) = std::pow(0.05, 2) * Eigen::Matrix3d::Identity(); // p
	covariance.block(6, 6, 3, 3) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity(); // v (static)

	return true;
}



dynamicInitializer::dynamicInitializer(const inertialInitializerOptions &initializer_options_, std::shared_ptr<featureDatabase> db_, 
	std::shared_ptr<std::vector<imuData>> imu_data_) : initializer_options(initializer_options_), db(db_), imu_data(imu_data_)
{

}

bool dynamicInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<baseType>> &order, std::shared_ptr<imuState> &imu_state_, 
	std::map<double, std::shared_ptr<poseJpl>> &clones_imu_, std::unordered_map<size_t, std::shared_ptr<mapPoint>> &map_points_)
{
	auto rT1 = boost::posix_time::microsec_clock::local_time();
	double newest_cam_time = -1;
	for (auto const &feat : db->getInternalData())
	{
		for (auto const &cam_time_pair : feat.second->timestamps)
		{
			for (auto const &time : cam_time_pair.second)
			{
				newest_cam_time = std::max(newest_cam_time, time);
			}
		}
	}

	double oldest_time = newest_cam_time - initializer_options.init_window_time;

	if (newest_cam_time < 0 || oldest_time < 0) return false;

	db->cleanUpOldmeasurements(oldest_time);
  
	bool have_old_imu_readings = false;
	auto it_imu = imu_data->begin();

	while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + initializer_options.calib_camimu_dt)
	{
		have_old_imu_readings = true;
		it_imu = imu_data->erase(it_imu);
	}

	if (db->getInternalData().size() < 0.75 * initializer_options.init_max_features)
	{
		std::cout << std::fixed << "[initialize-dynamic]: Only " << db->getInternalData().size() << " valid features of required (" 
			<< 0.95 * initializer_options.init_max_features << " thresh)" << std::endl;
		return false;
	}

	if (imu_data->size() < 2 || !have_old_imu_readings)
	{
		std::cout << std::fixed << "[initialize-dynamic]: Waiting for window to reach full size (" << imu_data->size() << " imu readings)" << std::endl;
		return false;
	}

	std::unordered_map<size_t, std::shared_ptr<feature>> features;

	for (const auto &feat : db->getInternalData())
	{
		auto feat_new = std::make_shared<feature>();
		feat_new->feature_id = feat.second->feature_id;
		feat_new->uvs = feat.second->uvs;
		feat_new->uvs_norm = feat.second->uvs_norm;
		feat_new->timestamps = feat.second->timestamps;
		features.insert({feat.first, feat_new});
	}

	const int min_num_meas_to_optimize = (int)initializer_options.init_window_time;
	const int min_valid_features = 8;

	bool have_stereo = false;
	int count_valid_features = 0;
	std::map<size_t, int> map_features_num_meas;
	int num_measurements = 0;
	double oldest_camera_time = INFINITY;
	std::map<double, bool> map_camera_times;
	map_camera_times[newest_cam_time] = true;
	std::map<size_t, bool> map_camera_ids;
	double pose_dt_avg = initializer_options.init_window_time / (double)(initializer_options.init_dyn_num_pose + 1);

	for (auto const &feat : features)
	{
		std::vector<double> times;
		std::map<size_t, bool> cam_ids;

		for (auto const &cam_time : feat.second->timestamps)
		{
			for (double time : cam_time.second)
			{
				double time_dt = INFINITY;

				for (auto const &tmp : map_camera_times)
					time_dt = std::min(time_dt, std::abs(time - tmp.first));

				for (auto const &tmp : times)
					time_dt = std::min(time_dt, std::abs(time - tmp));

				if (time_dt >= pose_dt_avg || time_dt == 0.0)
				{
					times.push_back(time);
					cam_ids[cam_time.first] = true;
				}
			}
		}

		map_features_num_meas[feat.first] = (int)times.size();
		if (map_features_num_meas[feat.first] < min_num_meas_to_optimize) continue;

		for (auto const &tmp : times)
		{
			map_camera_times[tmp] = true;
			oldest_camera_time = std::min(oldest_camera_time, tmp);
			num_measurements += 2;
		}

		for (auto const &tmp : cam_ids)
			map_camera_ids[tmp.first] = true;

		if (cam_ids.size() > 1) have_stereo = true;

		count_valid_features++;
	}

	if ((int)map_camera_times.size() < initializer_options.init_dyn_num_pose)
		return false;

	if (count_valid_features < min_valid_features)
	{
		std::cout << "[initialize-dynamic]: Only " << count_valid_features << " valid features of required " << min_valid_features << std::endl;
		return false;
	}

	Eigen::Vector3d gyroscope_bias = initializer_options.init_dyn_bias_g;
	Eigen::Vector3d accelerometer_bias = initializer_options.init_dyn_bias_a;

	double accel_inI_norm = 0.0;
	double theta_inI_norm = 0.0;
	double time_0_in_imu = oldest_camera_time + initializer_options.calib_camimu_dt;
	double time_1_in_imu = newest_cam_time + initializer_options.calib_camimu_dt;
	std::vector<imuData> readings = initializerHelper::selectImuReadings(*imu_data, time_0_in_imu, time_1_in_imu);

	assert(readings.size() > 2);
	for (size_t k = 0; k < readings.size() - 1; k++)
	{
		auto imu_0 = readings.at(k);
		auto imu_1 = readings.at(k + 1);
		double dt = imu_1.timestamp - imu_0.timestamp;
		Eigen::Vector3d gyr = 0.5 * (imu_0.gyr + imu_1.gyr) - gyroscope_bias;
		Eigen::Vector3d acc = 0.5 * (imu_0.acc + imu_1.acc) - accelerometer_bias;
		theta_inI_norm += ( -gyr * dt).norm();
		accel_inI_norm += acc.norm();
	}

	accel_inI_norm /= (double)(readings.size() - 1);

	if (180.0 / M_PI * theta_inI_norm < initializer_options.init_dyn_min_deg)
	{
		std::cout << "[initialize-dynamic]: Gyroscope only " << 180.0 / M_PI * theta_inI_norm << " degree change (" 
			<< initializer_options.init_dyn_min_deg << " thresh)" << std::endl;
		return false;
	}
	std::cout << std::fixed << "[initialize-dynamic]: |theta_I| = " << 180.0 / M_PI * theta_inI_norm 
		<< " deg and |accel| = " << accel_inI_norm << std::endl;

 	auto rT2 = boost::posix_time::microsec_clock::local_time();

	const bool use_single_depth = false;
	int size_feature = (use_single_depth) ? 1 : 3;
	int num_features = count_valid_features;
	int system_size = size_feature * num_features + 3 + 3;

	if (num_measurements < system_size)
	{
		std::cout << "[initialize-dynamic]: Not enough feature measurements (" << num_measurements << " meas vs " << system_size 
			<< " state size)" << std::endl;

		return false;
	}

	assert(oldest_camera_time < newest_cam_time);
	double last_camera_timestamp = 0.0;
	std::map<double, std::shared_ptr<preIntegrationV1>> map_camera_pre_integration_I0toIi, map_camera_pre_integration_IitoIi1; //map_camera_cpi_I0toIi, map_camera_cpi_IitoIi1;
	for (auto const &time_pair : map_camera_times)
	{
		double current_time = time_pair.first;
    
		if (current_time == oldest_camera_time)
		{
			map_camera_pre_integration_I0toIi.insert({current_time, nullptr});
			map_camera_pre_integration_IitoIi1.insert({current_time, nullptr});
			last_camera_timestamp = current_time;
			continue;
		}

		double pre_integration_I0toIi1_time_0_in_imu = oldest_camera_time + initializer_options.calib_camimu_dt;
		double pre_integration_I0toIi1_time_1_in_imu = current_time + initializer_options.calib_camimu_dt;
		auto pre_integration_I0toIi1 = std::make_shared<preIntegrationV1>(initializer_options.sigma_w, initializer_options.sigma_wb, initializer_options.sigma_a, initializer_options.sigma_ab, true);
		pre_integration_I0toIi1->setLinearizationPoints(gyroscope_bias, accelerometer_bias);

		std::vector<imuData> pre_integration_I0toIi1_readings =
			initializerHelper::selectImuReadings(*imu_data, pre_integration_I0toIi1_time_0_in_imu, pre_integration_I0toIi1_time_1_in_imu);
    
		if (pre_integration_I0toIi1_readings.size() < 2)
		{
			std::cout << std::fixed << "[initialize-dynamic]: Camera " << (pre_integration_I0toIi1_time_1_in_imu - pre_integration_I0toIi1_time_0_in_imu) 
				<< " in has " << pre_integration_I0toIi1_readings.size() << " IMU readings" << std::endl;
			return false;
		}

		double pre_integration_I0toIi1_dt_imu = pre_integration_I0toIi1_readings.at(pre_integration_I0toIi1_readings.size() - 1).timestamp - pre_integration_I0toIi1_readings.at(0).timestamp;
    
		if (std::abs(pre_integration_I0toIi1_dt_imu - (pre_integration_I0toIi1_time_1_in_imu - pre_integration_I0toIi1_time_0_in_imu)) > 0.01)
		{
			std::cout << std::fixed << "[initialize-dynamic]: Camera IMU was only propagated " << pre_integration_I0toIi1_dt_imu << " of " 
				<< (pre_integration_I0toIi1_time_1_in_imu - pre_integration_I0toIi1_time_0_in_imu) << std::endl;
			return false;
		}

		for (size_t k = 0; k < pre_integration_I0toIi1_readings.size() - 1; k++)
		{
			auto imu_0 = pre_integration_I0toIi1_readings.at(k);
			auto imu_1 = pre_integration_I0toIi1_readings.at(k + 1);
			pre_integration_I0toIi1->feedImu(imu_0.timestamp, imu_1.timestamp, imu_0.gyr, imu_0.acc, imu_1.gyr, imu_1.acc);
		}

		double pre_integration_IitoIi1_time_0_in_imu = last_camera_timestamp + initializer_options.calib_camimu_dt;
		double pre_integration_IitoIi1_time_1_in_imu = current_time + initializer_options.calib_camimu_dt;
		auto pre_integration_IitoIi1 = std::make_shared<preIntegrationV1>(initializer_options.sigma_w, initializer_options.sigma_wb, 
			initializer_options.sigma_a, initializer_options.sigma_ab, true);
		pre_integration_IitoIi1->setLinearizationPoints(gyroscope_bias, accelerometer_bias);

		std::vector<imuData> pre_integration_IitoIi1_readings =
			initializerHelper::selectImuReadings(*imu_data, pre_integration_IitoIi1_time_0_in_imu, pre_integration_IitoIi1_time_1_in_imu);

		if (pre_integration_IitoIi1_readings.size() < 2)
		{
			std::cout << std::fixed << "[initialize-dynamic]: Camera " << (pre_integration_IitoIi1_time_1_in_imu - pre_integration_IitoIi1_time_0_in_imu) << " in has " 
				<< pre_integration_IitoIi1_readings.size() << " IMU readings" << std::endl;
			return false;
		}

		double pre_integration_IitoIi1_dt_imu = pre_integration_IitoIi1_readings.at(pre_integration_IitoIi1_readings.size() - 1).timestamp - pre_integration_IitoIi1_readings.at(0).timestamp;
    
		if (std::abs(pre_integration_IitoIi1_dt_imu - (pre_integration_IitoIi1_time_1_in_imu - pre_integration_IitoIi1_time_0_in_imu)) > 0.01)
		{
			std::cout << std::fixed << "[initialize-dynamic]: Camera IMU was only propagated " << pre_integration_IitoIi1_dt_imu << " of " 
				<< (pre_integration_IitoIi1_time_1_in_imu - pre_integration_IitoIi1_time_0_in_imu) << std::endl;
			return false;
		}

		for (size_t k = 0; k < pre_integration_IitoIi1_readings.size() - 1; k++)
		{
			auto imu_0 = pre_integration_IitoIi1_readings.at(k);
			auto imu_1 = pre_integration_IitoIi1_readings.at(k + 1);
			pre_integration_IitoIi1->feedImu(imu_0.timestamp, imu_1.timestamp, imu_0.gyr, imu_0.acc, imu_1.gyr, imu_1.acc);
		}

		map_camera_pre_integration_I0toIi.insert({current_time, pre_integration_I0toIi1});
		map_camera_pre_integration_IitoIi1.insert({current_time, pre_integration_IitoIi1});
		last_camera_timestamp = current_time;
	}

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_measurements, system_size);
	Eigen::VectorXd b = Eigen::VectorXd::Zero(num_measurements);
	std::cout << "[initialize-dynamic]: System of " << num_measurements << " measurement x " << system_size << " states created (" << num_features 
		<< " features, " << ((have_stereo) ? "stereo" : "mono") << ")" << std::endl;

	int index_meas = 0;
	int idx_feat = 0;
	std::map<size_t, int> A_index_features;

	for (auto const &feat : features)
	{
		if (map_features_num_meas[feat.first] < min_num_meas_to_optimize) continue;

		if (A_index_features.find(feat.first) == A_index_features.end())
		{
			A_index_features.insert({feat.first, idx_feat});
			idx_feat += 1;
		}

		for (auto const &cam_time : feat.second->timestamps)
		{
			size_t cam_id = cam_time.first;
			Eigen::Vector4d q_ItoC = initializer_options.camera_extrinsics.at(cam_id).block(0, 0, 4, 1);
			Eigen::Vector3d p_IinC = initializer_options.camera_extrinsics.at(cam_id).block(4, 0, 3, 1);
			Eigen::Matrix3d R_ItoC = quatType::quatToRot(q_ItoC);

			for (size_t i = 0; i < cam_time.second.size(); i++)
			{
				double time = feat.second->timestamps.at(cam_id).at(i);

				if (map_camera_times.find(time) == map_camera_times.end()) continue;

				Eigen::Vector2d uv_norm;
				uv_norm << (double)feat.second->uvs_norm.at(cam_id).at(i)(0), (double)feat.second->uvs_norm.at(cam_id).at(i)(1);

				double dt = 0.0;
				Eigen::MatrixXd R_I0toIk = Eigen::MatrixXd::Identity(3, 3);
				Eigen::MatrixXd alpha_I0toIk = Eigen::MatrixXd::Zero(3, 1);
				if (map_camera_pre_integration_I0toIi.find(time) != map_camera_pre_integration_I0toIi.end() && map_camera_pre_integration_I0toIi.at(time) != nullptr)
				{
					dt = map_camera_pre_integration_I0toIi.at(time)->dt;
					R_I0toIk = map_camera_pre_integration_I0toIi.at(time)->R_tau_k;
					alpha_I0toIk = map_camera_pre_integration_I0toIi.at(time)->alpha_tau;
				}

				Eigen::MatrixXd H_proj = Eigen::MatrixXd::Zero(2, 3);
				H_proj << 1, 0, -uv_norm(0), 0, 1, -uv_norm(1);
				Eigen::MatrixXd Y = H_proj * R_ItoC * R_I0toIk;
				Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(2, system_size);
				Eigen::MatrixXd b_i = Y * alpha_I0toIk - H_proj * p_IinC;

				if (size_feature == 1)
				{
					assert(false);
				}
				else
				{
					H_i.block(0, size_feature * A_index_features.at(feat.first), 2, 3) = Y;
				}

				H_i.block(0, size_feature * num_features + 0, 2, 3) = -dt * Y;
				H_i.block(0, size_feature * num_features + 3, 2, 3) = 0.5 * dt * dt * Y;

				A.block(index_meas, 0, 2, A.cols()) = H_i;
				b.block(index_meas, 0, 2, 1) = b_i;
				index_meas += 2;
			}
		}
	}
	auto rT3 = boost::posix_time::microsec_clock::local_time();

	Eigen::MatrixXd A1 = A.block(0, 0, A.rows(), A.cols() - 3);
	Eigen::MatrixXd A1A1_inv = (A1.transpose() * A1).llt().solve(Eigen::MatrixXd::Identity(A1.cols(), A1.cols()));
	Eigen::MatrixXd A2 = A.block(0, A.cols() - 3, A.rows(), 3);
	Eigen::MatrixXd Temp = A2.transpose() * (Eigen::MatrixXd::Identity(A1.rows(), A1.rows()) - A1 * A1A1_inv * A1.transpose());
	Eigen::MatrixXd D = Temp * A2;
	Eigen::MatrixXd d = Temp * b;
	Eigen::Matrix<double, 7, 1> coeff = initializerHelper::computeDongsiCoeff(D, d, initializer_options.gravity_mag);

	assert(coeff(0) == 1);
	Eigen::Matrix<double, 6, 6> companion_matrix = Eigen::Matrix<double, 6, 6>::Zero(coeff.rows() - 1, coeff.rows() - 1);
	companion_matrix.diagonal(-1).setOnes();
	companion_matrix.col(companion_matrix.cols() - 1) = - coeff.reverse().head(coeff.rows() - 1);
	Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd_0(companion_matrix);
	Eigen::MatrixXd singular_values_0 = svd_0.singularValues();
	double cond_0 = singular_values_0(0) / singular_values_0(singular_values_0.rows() - 1);

	std::cout << std::fixed << "[initialize-dynamic]: CM cond = " << cond_0 << " | rank = " << (int)svd_0.rank() << " of " << (int)companion_matrix.cols() 
		<< " (" << svd_0.threshold() << " thresh)" << std::endl;

	if (svd_0.rank() != companion_matrix.rows())
	{
		std::cout << "[initialize-dynamic]: Eigenvalue decomposition not full rank" << std::endl;
		return false;
	}

	Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> solver(companion_matrix, false);
  
	if (solver.info() != Eigen::Success)
	{
		std::cout << "[initialize-dynamic]: failed to compute the eigenvalue decomposition" << std::endl;
		return false;
	}

	bool lambda_found = false;
	double lambda_min = -1;
	double cost_min = INFINITY;
	Eigen::MatrixXd I_dd = Eigen::MatrixXd::Identity(D.rows(), D.rows());

	for (int i = 0; i < solver.eigenvalues().size(); i++) {
		auto val = solver.eigenvalues()(i);
		if (val.imag() == 0)
		{
			double lambda = val.real();
			Eigen::MatrixXd D_lambdaI_inv = (D - lambda * I_dd).llt().solve(I_dd);
			Eigen::VectorXd state_grav = D_lambdaI_inv * d;
			double cost = std::abs(state_grav.norm() - initializer_options.gravity_mag);

			if (!lambda_found || cost < cost_min)
			{
				lambda_found = true;
				lambda_min = lambda;
				cost_min = cost;
			}
		}
	}

	if (!lambda_found)
	{
		std::cout << "[initialize-dynamic]: Failed to find a real eigenvalue" << std::endl;
		return false;
	}

	std::cout << std::fixed << "[initialize-dynamic]: Smallest real eigenvalue = " << lambda_min << " (cost of " 
		<< cost_min << ")" << std::endl;

	Eigen::MatrixXd D_lambdaI_inv = (D - lambda_min * I_dd).llt().solve(I_dd);
	Eigen::VectorXd state_grav = D_lambdaI_inv * d;

	Eigen::VectorXd state_feat_vel = - A1A1_inv * A1.transpose() * A2 * state_grav + A1A1_inv * A1.transpose() * b;
	Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(system_size, 1);
	x_hat.block(0, 0, size_feature * num_features + 3, 1) = state_feat_vel;
	x_hat.block(size_feature * num_features + 3, 0, 3, 1) = state_grav;
	Eigen::Vector3d v_I0inI0 = x_hat.block(size_feature * num_features + 0, 0, 3, 1);
	std::cout << std::fixed << "[initialize-dynamic]: Velocity in I0 was " << v_I0inI0(0) << ", " << v_I0inI0(1) << ", " << v_I0inI0(2) 
		<< " and |v| = " << v_I0inI0.norm() << std::endl;

	Eigen::Vector3d gravity_inI0 = x_hat.block(size_feature * num_features + 3, 0, 3, 1);
	double init_max_grav_difference = 1e-3;

	if (std::abs(gravity_inI0.norm() - initializer_options.gravity_mag) > init_max_grav_difference)
	{
		std::cout << std::fixed << "[initialize-dynamic]: Gravity did not converge (" << std::abs(gravity_inI0.norm() - initializer_options.gravity_mag) 
			<< " > " << init_max_grav_difference << ")" << std::endl;
		return false;
	}

	std::cout << "[initialize-dynamic]: Gravity in I0 was " << gravity_inI0(0) << ", " << gravity_inI0(1) << ", " << gravity_inI0(2) << " and |g| = " << gravity_inI0.norm() << std::endl;
	auto rT4 = boost::posix_time::microsec_clock::local_time();

	std::map<double, Eigen::VectorXd> ori_I0toIi, pos_IiinI0, vel_IiinI0;
	
	for (auto const &time_pair : map_camera_times)
	{
		double time = time_pair.first;

		double dt = 0.0;
		Eigen::MatrixXd R_I0toIk = Eigen::MatrixXd::Identity(3, 3);
		Eigen::MatrixXd alpha_I0toIk = Eigen::MatrixXd::Zero(3, 1);
		Eigen::MatrixXd beta_I0toIk = Eigen::MatrixXd::Zero(3, 1);

		if (map_camera_pre_integration_I0toIi.find(time) != map_camera_pre_integration_I0toIi.end() && map_camera_pre_integration_I0toIi.at(time) != nullptr)
		{
			auto pre_integration = map_camera_pre_integration_I0toIi.at(time);
			dt = pre_integration->dt;
			R_I0toIk = pre_integration->R_tau_k;
			alpha_I0toIk = pre_integration->alpha_tau;
			beta_I0toIk = pre_integration->beta_tau;
		}

		Eigen::Vector3d p_IkinI0 = v_I0inI0 * dt - 0.5 * gravity_inI0 * dt * dt + alpha_I0toIk;
		Eigen::Vector3d v_IkinI0 = v_I0inI0 - gravity_inI0 * dt + beta_I0toIk;

		ori_I0toIi.insert({time, quatType::rotToQuat(R_I0toIk)});
		pos_IiinI0.insert({time, p_IkinI0});
		vel_IiinI0.insert({time, v_IkinI0});
	}

	count_valid_features = 0;
	std::map<size_t, Eigen::Vector3d> features_inI0;
	for (auto const &feat : features)
	{
		if (map_features_num_meas[feat.first] < min_num_meas_to_optimize) continue;

		Eigen::Vector3d p_FinI0;
		if (size_feature == 1)
			assert(false);
		else
			p_FinI0 = x_hat.block(size_feature * A_index_features.at(feat.first), 0, 3, 1);

		bool is_behind = false;
		
		for (auto const &cam_time : feat.second->timestamps)
		{
			size_t cam_id = cam_time.first;
			Eigen::Vector4d q_ItoC = initializer_options.camera_extrinsics.at(cam_id).block(0, 0, 4, 1);
			Eigen::Vector3d p_IinC = initializer_options.camera_extrinsics.at(cam_id).block(4, 0, 3, 1);
			Eigen::Vector3d p_FinC0 = quatType::quatToRot(q_ItoC) * p_FinI0 + p_IinC;

			if (p_FinC0(2) < 0) is_behind = true;
		}
		
		if (!is_behind)
		{
			features_inI0.insert({feat.first, p_FinI0});
			count_valid_features++;
		}
	}

	if (count_valid_features < min_valid_features)
	{
		std::cout << std::fixed << "[initialize-dynamic]: Not enough features for our mle (" << count_valid_features << " < " 
			<< min_valid_features << ")" << std::endl;
		return false;
	}

	Eigen::Matrix3d R_GtoI0;
	initializerHelper::gramSchmidt(gravity_inI0, R_GtoI0);
	Eigen::Vector4d q_GtoI0 = quatType::rotToQuat(R_GtoI0);
	Eigen::Vector3d gravity;
	gravity << 0.0, 0.0, initializer_options.gravity_mag;
	std::map<double, Eigen::VectorXd> ori_GtoIi, pos_IiinG, vel_IiinG;
	std::map<size_t, Eigen::Vector3d> features_inG;

	for (auto const &time_pair : map_camera_times)
	{
		ori_GtoIi[time_pair.first] = quatType::quatMultiply(ori_I0toIi.at(time_pair.first), q_GtoI0);
		pos_IiinG[time_pair.first] = R_GtoI0.transpose() * pos_IiinI0.at(time_pair.first);
		vel_IiinG[time_pair.first] = R_GtoI0.transpose() * vel_IiinI0.at(time_pair.first);
	}

	for (auto const &feat : features_inI0)
	features_inG[feat.first] = R_GtoI0.transpose() * feat.second;

	ceres::Problem problem;

	std::map<double, int> map_states;
	std::vector<double*> ceres_vars_ori;
	std::vector<double*> ceres_vars_pos;
	std::vector<double*> ceres_vars_vel;
	std::vector<double*> ceres_vars_bias_g;
	std::vector<double*> ceres_vars_bias_a;

	std::map<size_t, int> map_features;
	std::vector<double*> ceres_vars_feat;

	std::map<size_t, int> map_calib_cam2imu;
	std::vector<double*> ceres_vars_calib_cam2imu_ori;
	std::vector<double*> ceres_vars_calib_cam2imu_pos;

	std::map<size_t, int> map_calib_cam;
	std::vector<double*> ceres_vars_calib_cam_intrinsics;

	auto freeStateMemory = [&]()
	{
		for (auto const &ptr : ceres_vars_ori)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_pos)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_vel)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_bias_g)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_bias_a)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_feat)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_calib_cam2imu_ori)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_calib_cam2imu_pos)
			delete[] ptr;
		for (auto const &ptr : ceres_vars_calib_cam_intrinsics)
			delete[] ptr;
	};

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.trust_region_strategy_type = ceres::DOGLEG;
	options.num_threads = initializer_options.init_dyn_mle_max_threads;
	options.max_solver_time_in_seconds = initializer_options.init_dyn_mle_max_time;
	options.max_num_iterations = initializer_options.init_dyn_mle_max_iter;
	options.function_tolerance = 1e-5;
	options.gradient_tolerance = 1e-4 * options.function_tolerance;

	double timestamp_k = -1;

	for (auto const &time_pair : map_camera_times)
	{
		double timestamp_k1 = time_pair.first;
		std::shared_ptr<preIntegrationV1> pre_integration = map_camera_pre_integration_IitoIi1.at(timestamp_k1);
		Eigen::Matrix<double, 16, 1> state_k1;
		state_k1.block(0, 0, 4, 1) = ori_GtoIi.at(timestamp_k1);
		state_k1.block(4, 0, 3, 1) = pos_IiinG.at(timestamp_k1);
		state_k1.block(7, 0, 3, 1) = vel_IiinG.at(timestamp_k1);
		state_k1.block(10, 0, 3, 1) = gyroscope_bias;
		state_k1.block(13, 0, 3, 1) = accelerometer_bias;

		auto *var_ori = new double[4];
		for (int j = 0; j < 4; j++)
			var_ori[j] = state_k1(0 + j, 0);

		auto *var_pos = new double[3];
		auto *var_vel = new double[3];
		auto *var_bias_g = new double[3];
		auto *var_bias_a = new double[3];

		for (int j = 0; j < 3; j++)
		{
			var_pos[j] = state_k1(4 + j, 0);
			var_vel[j] = state_k1(7 + j, 0);
			var_bias_g[j] = state_k1(10 + j, 0);
			var_bias_a[j] = state_k1(13 + j, 0);
		}

		auto ceres_jplquat = new stateJplQuatLocal();
		problem.AddParameterBlock(var_ori, 4, ceres_jplquat);
		problem.AddParameterBlock(var_pos, 3);
		problem.AddParameterBlock(var_vel, 3);
		problem.AddParameterBlock(var_bias_g, 3);
		problem.AddParameterBlock(var_bias_a, 3);

		if (map_states.empty())
		{
			Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(13, 1);

			for (int j = 0; j < 4; j++)
				x_lin(0 + j) = var_ori[j];

			for (int j = 0; j < 3; j++)
			{
				x_lin(4 + j) = var_pos[j];
				x_lin(7 + j) = var_bias_g[j];
				x_lin(10 + j) = var_bias_a[j];
			}
			
			Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(10, 1);
			Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(10, 10);
			prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1e-5, 2);
			prior_Info.block(4, 4, 3, 3) *= 1.0 / std::pow(0.05, 2);
			prior_Info.block(7, 7, 3, 3) *= 1.0 / std::pow(0.10, 2);

			std::vector<std::string> x_types;
			std::vector<double *> factor_params;
			factor_params.push_back(var_ori);
			x_types.emplace_back("quat_yaw");
			factor_params.push_back(var_pos);
			x_types.emplace_back("vec3");
			factor_params.push_back(var_bias_g);
			x_types.emplace_back("vec3");
			factor_params.push_back(var_bias_a);
			x_types.emplace_back("vec3");

			auto *factor_prior = new factorGenericPrior(x_lin, x_types, prior_Info, prior_grad);
			problem.AddResidualBlock(factor_prior, nullptr, factor_params);
		}

		map_states.insert({timestamp_k1, (int)ceres_vars_ori.size()});
		ceres_vars_ori.push_back(var_ori);
		ceres_vars_pos.push_back(var_pos);
		ceres_vars_vel.push_back(var_vel);
		ceres_vars_bias_g.push_back(var_bias_g);
		ceres_vars_bias_a.push_back(var_bias_a);

		if (pre_integration != nullptr)
		{
			assert(timestamp_k != -1);
			std::vector<double *> factor_params;
			factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k)));
			factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k)));
			factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k)));
			factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k)));
			factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k)));
			factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k1)));
			factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k1)));
			factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k1)));
			factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k1)));
			factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k1)));
			auto *factor_imu = new factorImuPreIntegrationV1(pre_integration->dt, gravity, pre_integration->alpha_tau, pre_integration->beta_tau, 
				pre_integration->q_tau_k, pre_integration->b_a_lin, pre_integration->b_w_lin, pre_integration->J_q, pre_integration->J_b, 
				pre_integration->J_a, pre_integration->H_b, pre_integration->H_a, pre_integration->P_meas);
			
			problem.AddResidualBlock(factor_imu, nullptr, factor_params);
		}

		timestamp_k = timestamp_k1;
	}

	for (auto const &id_pair : map_camera_ids)
	{
		size_t cam_id = id_pair.first;
		if (map_calib_cam2imu.find(cam_id) == map_calib_cam2imu.end())
		{
			auto *var_calib_ori = new double[4];
			for (int j = 0; j < 4; j++)
				var_calib_ori[j] = initializer_options.camera_extrinsics.at(cam_id)(0 + j, 0);

			auto *var_calib_pos = new double[3];
			for (int j = 0; j < 3; j++)
				var_calib_pos[j] = initializer_options.camera_extrinsics.at(cam_id)(4 + j, 0);

			auto ceres_calib_jplquat = new stateJplQuatLocal();
			problem.AddParameterBlock(var_calib_ori, 4, ceres_calib_jplquat);
			problem.AddParameterBlock(var_calib_pos, 3);

			map_calib_cam2imu.insert({cam_id, (int)ceres_vars_calib_cam2imu_ori.size()});
			ceres_vars_calib_cam2imu_ori.push_back(var_calib_ori);
			ceres_vars_calib_cam2imu_pos.push_back(var_calib_pos);

			Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(7, 1);
			for (int j = 0; j < 4; j++)
				x_lin(0 + j) = var_calib_ori[j];

			for (int j = 0; j < 3; j++)
				x_lin(4 + j) = var_calib_pos[j];

			Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(6, 1);
			Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(6, 6);
			prior_Info.block(0, 0, 3, 3) *= 1.0 / std::pow(0.001, 2);
			prior_Info.block(3, 3, 3, 3) *= 1.0 / std::pow(0.01, 2);

			std::vector<std::string> x_types;
			std::vector<double*> factor_params;
			factor_params.push_back(var_calib_ori);
			x_types.emplace_back("quat");
			factor_params.push_back(var_calib_pos);
			x_types.emplace_back("vec3");

			auto *factor_prior = new factorGenericPrior(x_lin, x_types, prior_Info, prior_grad);
			problem.AddResidualBlock(factor_prior, nullptr, factor_params);

			if (!initializer_options.init_dyn_mle_opt_calib)
			{
				problem.SetParameterBlockConstant(var_calib_ori);
				problem.SetParameterBlockConstant(var_calib_pos);
			}
		}

		if (map_calib_cam.find(cam_id) == map_calib_cam.end())
		{
			auto *var_calib_cam = new double[8];
			for (int j = 0; j < 8; j++)
				var_calib_cam[j] = initializer_options.camera_intrinsics.at(cam_id)->getValue()(j, 0);

			problem.AddParameterBlock(var_calib_cam, 8);
			map_calib_cam.insert({cam_id, (int)ceres_vars_calib_cam_intrinsics.size()});
			ceres_vars_calib_cam_intrinsics.push_back(var_calib_cam);

			Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(8, 1);
			for (int j = 0; j < 8; j++)
				x_lin(0 + j) = var_calib_cam[j];

			Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(8, 1);
			Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(8, 8);
			prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1.0, 2);
			prior_Info.block(4, 4, 4, 4) *= 1.0 / std::pow(0.005, 2);

			std::vector<std::string> x_types;
			std::vector<double*> factor_params;
			factor_params.push_back(var_calib_cam);
			x_types.emplace_back("vec8");

			auto *factor_prior = new factorGenericPrior(x_lin, x_types, prior_Info, prior_grad);
			problem.AddResidualBlock(factor_prior, nullptr, factor_params);

			if (!initializer_options.init_dyn_mle_opt_calib)
				problem.SetParameterBlockConstant(var_calib_cam);
		}
	}
	assert(map_calib_cam2imu.size() == map_calib_cam.size());

	for (auto const &feat : features)
	{
		if (map_features_num_meas[feat.first] < min_num_meas_to_optimize) continue;

		if (features_inG.find(feat.first) == features_inG.end()) continue;

		for (auto const &cam_time : feat.second->timestamps)
		{
			size_t feat_id = feat.first;
			size_t cam_id = cam_time.first;
			bool is_fisheye = (std::dynamic_pointer_cast<cameraEqui>(initializer_options.camera_intrinsics.at(cam_id)) != nullptr);

			for (size_t i = 0; i < cam_time.second.size(); i++)
			{
				double time = feat.second->timestamps.at(cam_id).at(i);
				if (map_camera_times.find(time) == map_camera_times.end()) continue;

				Eigen::Vector2d uv_raw = feat.second->uvs.at(cam_id).at(i).block(0, 0, 2, 1).cast<double>();

				if (map_features.find(feat_id) == map_features.end())
				{
					auto *var_feat = new double[3];
					for (int j = 0; j < 3; j++)
						var_feat[j] = features_inG.at(feat_id)(j);

					problem.AddParameterBlock(var_feat, 3);
					map_features.insert({feat_id, (int)ceres_vars_feat.size()});
					ceres_vars_feat.push_back(var_feat);
				}

				std::vector<double *> factor_params;
				factor_params.push_back(ceres_vars_ori.at(map_states.at(time)));
				factor_params.push_back(ceres_vars_pos.at(map_states.at(time)));
				factor_params.push_back(ceres_vars_feat.at(map_features.at(feat_id)));
				factor_params.push_back(ceres_vars_calib_cam2imu_ori.at(map_calib_cam2imu.at(cam_id)));
				factor_params.push_back(ceres_vars_calib_cam2imu_pos.at(map_calib_cam2imu.at(cam_id)));
				factor_params.push_back(ceres_vars_calib_cam_intrinsics.at(map_calib_cam.at(cam_id)));
				auto *factor_pinhole = new factorImageReprojCalib(uv_raw, initializer_options.sigma_pix, is_fisheye);

				ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
				problem.AddResidualBlock(factor_pinhole, loss_function, factor_params);
			}
		}
	}
	assert(ceres_vars_ori.size() == ceres_vars_bias_g.size());
	assert(ceres_vars_ori.size() == ceres_vars_vel.size());
	assert(ceres_vars_ori.size() == ceres_vars_bias_a.size());
	assert(ceres_vars_ori.size() == ceres_vars_pos.size());
	auto rT5 = boost::posix_time::microsec_clock::local_time();

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << "[initialize-dynamic]: " << (int)summary.iterations.size() << " iterations | " << map_states.size() << " states, " 
		<< map_features.size() << " feats (" << count_valid_features << " valid) | " << summary.num_parameters << " param and "
		<< summary.num_residuals << " res | cost " << std::fixed << summary.initial_cost << " => " << summary.final_cost << std::endl;

	auto rT6 = boost::posix_time::microsec_clock::local_time();

	timestamp = newest_cam_time;
	if (initializer_options.init_dyn_mle_max_iter != 0 && summary.termination_type != ceres::CONVERGENCE)
	{
		std::cout << "[initialize-dynamic]: Optimization failed: " << summary.message << std::endl;
		freeStateMemory();
		return false;
	}
	std::cout << "[initialize-dynamic]: " << summary.message << std::endl;

	auto getPose = [&](double timestamp)
	{
		Eigen::VectorXd state_imu = Eigen::VectorXd::Zero(16);

		for (int i = 0; i < 4; i++)
			state_imu(0 + i) = ceres_vars_ori[map_states[timestamp]][i];

		for (int i = 0; i < 3; i++)
		{
			state_imu(4 + i) = ceres_vars_pos[map_states[timestamp]][i];
			state_imu(7 + i) = ceres_vars_vel[map_states[timestamp]][i];
			state_imu(10 + i) = ceres_vars_bias_g[map_states[timestamp]][i];
			state_imu(13 + i) = ceres_vars_bias_a[map_states[timestamp]][i];
		}

		return state_imu;
	};

	assert(map_states.find(newest_cam_time) != map_states.end());
	
	if (imu_state_ == nullptr)
		imu_state_ = std::make_shared<imuState>();

	Eigen::VectorXd imu_state = getPose(newest_cam_time);
	imu_state_->setValue(imu_state);
	imu_state_->setFej(imu_state);

	for (auto const &state_pair : map_states)
	{
		Eigen::VectorXd pose = getPose(state_pair.first);

		if (clones_imu_.find(state_pair.first) == clones_imu_.end())
		{
			auto pose_ = std::make_shared<poseJpl>();
			pose_->setValue(pose.block(0, 0, 7, 1));
			pose_->setFej(pose.block(0, 0, 7, 1));
			clones_imu_.insert({state_pair.first, pose_});
		}
		else
		{
			clones_imu_.at(state_pair.first)->setValue(pose.block(0, 0, 7, 1));
			clones_imu_.at(state_pair.first)->setFej(pose.block(0, 0, 7, 1));
		}
	}

	for (auto const &feat_pair : map_features)
	{
		Eigen::Vector3d feature;
		feature << ceres_vars_feat[feat_pair.second][0], ceres_vars_feat[feat_pair.second][1], ceres_vars_feat[feat_pair.second][2];

		if (map_points_.find(feat_pair.first) == map_points_.end())
		{
			auto feature_ = std::make_shared<mapPoint>();
			feature_->feature_id = feat_pair.first;
			feature_->setPointXYZ(feature, false);
			feature_->setPointXYZ(feature, true);
			map_points_.insert({feat_pair.first, feature_});
		}
		else
		{
			map_points_.at(feat_pair.first)->feature_id = feat_pair.first;
			map_points_.at(feat_pair.first)->setPointXYZ(feature, false);
			map_points_.at(feat_pair.first)->setPointXYZ(feature, true);
		}
	}

	std::vector<std::pair<const double *, const double *>> covariance_blocks;
	int state_index = map_states[newest_cam_time];

	covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_ori[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_pos[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_vel[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_bias_g[state_index], ceres_vars_bias_g[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_bias_a[state_index], ceres_vars_bias_a[state_index]));

	covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_pos[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_vel[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_bias_g[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_bias_a[state_index]));

	covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_vel[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_bias_g[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_bias_a[state_index]));

	covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_bias_g[state_index]));
	covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_bias_a[state_index]));

	covariance_blocks.push_back(std::make_pair(ceres_vars_bias_g[state_index], ceres_vars_bias_a[state_index]));

	ceres::Covariance::Options options_cov;
	options_cov.null_space_rank = (!initializer_options.init_dyn_mle_opt_calib) * ((int)map_calib_cam2imu.size() * (6 + 8));
	options_cov.min_reciprocal_condition_number = initializer_options.init_dyn_min_rec_cond;
  
	options_cov.apply_loss_function = true;
	options_cov.num_threads = initializer_options.init_dyn_mle_max_threads;

	ceres::Covariance problem_cov(options_cov);
	bool success = problem_cov.Compute(covariance_blocks, &problem);

	if (!success)
	{
		std::cout << "[initialize-dynamic]: Covariance recovery failed..." << std::endl;
		freeStateMemory();
		return false;
	}

	order.clear();
	order.push_back(imu_state_);
	covariance = Eigen::MatrixXd::Zero(imu_state_->getSize(), imu_state_->getSize());
	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov_tmp = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();

	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_ori[state_index], cov_tmp.data()));
	covariance.block(0, 0, 3, 3) = cov_tmp.eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_pos[state_index], cov_tmp.data()));
	covariance.block(3, 3, 3, 3) = cov_tmp.eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_vel[state_index], cov_tmp.data()));
	covariance.block(6, 6, 3, 3) = cov_tmp.eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_g[state_index], ceres_vars_bias_g[state_index], cov_tmp.data()));
	covariance.block(9, 9, 3, 3) = cov_tmp.eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_a[state_index], ceres_vars_bias_a[state_index], cov_tmp.data()));
	covariance.block(12, 12, 3, 3) = cov_tmp.eval();

	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_pos[state_index], cov_tmp.data()));
	covariance.block(0, 3, 3, 3) = cov_tmp.eval();
	covariance.block(3, 0, 3, 3) = cov_tmp.transpose();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_vel[state_index], cov_tmp.data()));
	covariance.block(0, 6, 3, 3) = cov_tmp.eval();
	covariance.block(6, 0, 3, 3) = cov_tmp.transpose().eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_bias_g[state_index], cov_tmp.data()));
	covariance.block(0, 9, 3, 3) = cov_tmp.eval();
	covariance.block(9, 0, 3, 3) = cov_tmp.transpose().eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_bias_a[state_index], cov_tmp.data()));
	covariance.block(0, 12, 3, 3) = cov_tmp.eval();
	covariance.block(12, 0, 3, 3) = cov_tmp.transpose().eval();

	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_vel[state_index], cov_tmp.data()));
	covariance.block(3, 6, 3, 3) = cov_tmp.eval();
	covariance.block(6, 3, 3, 3) = cov_tmp.transpose().eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_bias_g[state_index], cov_tmp.data()));
	covariance.block(3, 9, 3, 3) = cov_tmp.eval();
	covariance.block(9, 3, 3, 3) = cov_tmp.transpose().eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_bias_a[state_index], cov_tmp.data()));
	covariance.block(3, 12, 3, 3) = cov_tmp.eval();
	covariance.block(12, 3, 3, 3) = cov_tmp.transpose().eval();

	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_bias_g[state_index], cov_tmp.data()));
	covariance.block(6, 9, 3, 3) = cov_tmp.eval();
	covariance.block(9, 6, 3, 3) = cov_tmp.transpose().eval();
	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_bias_a[state_index], cov_tmp.data()));
	covariance.block(6, 12, 3, 3) = cov_tmp.eval();
	covariance.block(12, 6, 3, 3) = cov_tmp.transpose().eval();

	CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_g[state_index], ceres_vars_bias_a[state_index], cov_tmp.data()));
	covariance.block(9, 12, 3, 3) = cov_tmp.eval();
	covariance.block(12, 9, 3, 3) = cov_tmp.transpose().eval();

	covariance.block(0, 0, 3, 3) *= initializer_options.init_dyn_inflation_orientation;
	covariance.block(6, 6, 3, 3) *= initializer_options.init_dyn_inflation_velocity;
	covariance.block(9, 9, 3, 3) *= initializer_options.init_dyn_inflation_bias_gyro;
	covariance.block(12, 12, 3, 3) *= initializer_options.init_dyn_inflation_bias_accel;

	covariance = 0.5 * (covariance + covariance.transpose());
	Eigen::Vector3d sigmas_vel = covariance.block(6, 6, 3, 3).diagonal().transpose().cwiseSqrt();
	Eigen::Vector3d sigmas_bg = covariance.block(9, 9, 3, 3).diagonal().transpose().cwiseSqrt();
	Eigen::Vector3d sigmas_ba = covariance.block(12, 12, 3, 3).diagonal().transpose().cwiseSqrt();
	std::cout << "[initialize-dynamic]: vel priors = " << sigmas_vel(0) << ", " << sigmas_vel(1) << ", " << sigmas_vel(2) << std::endl;
	std::cout << "[initialize-dynamic]: bg priors = " << sigmas_bg(0) << ", " << sigmas_bg(1) << ", " << sigmas_bg(2) << std::endl;
	std::cout << "[initialize-dynamic]: ba priors = " << sigmas_ba(0) << ", " << sigmas_ba(1) << ", " << sigmas_ba(2) << std::endl;

	Eigen::MatrixXd x = imu_state_->value();
	x.block(4, 0, 3, 1).setZero();
	imu_state_->setValue(x);
	imu_state_->setFej(x);

	auto rT7 = boost::posix_time::microsec_clock::local_time();
	std::cout << std::fixed << "[initialize-dynamic]: " << (rT2 - rT1).total_microseconds() * 1e-6 << " sec for prelim tests" << std::endl;
	std::cout << std::fixed << "[initialize-dynamic]: " << (rT3 - rT2).total_microseconds() * 1e-6 << " sec for linsys setup" << std::endl;
	std::cout << std::fixed << "[initialize-dynamic]: " << (rT4 - rT3).total_microseconds() * 1e-6 << " sec for linsys" << std::endl;
	std::cout << std::fixed << "[initialize-dynamic]: " << (rT5 - rT4).total_microseconds() * 1e-6 << " sec for ceres opt setup" << std::endl;
	std::cout << std::fixed << "[initialize-dynamic]: " << (rT6 - rT5).total_microseconds() * 1e-6 << " sec for ceres opt" << std::endl;
	std::cout << std::fixed << "[initialize-dynamic]: " << (rT7 - rT6).total_microseconds() * 1e-6 << " sec for ceres covariance" << std::endl;
	std::cout << std::fixed << "[initialize-dynamic]: " << (rT7 - rT1).total_microseconds() * 1e-6 << " sec total for initialization" << std::endl;
	freeStateMemory();

	return true;
}