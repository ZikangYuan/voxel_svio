#include "sensorData.h"

static imuData interpolateData(const imuData &imu_1, const imuData &imu_2, double timestamp)
{
	double alpha = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);

	imuData data;
	data.timestamp = timestamp;
	data.acc = (1 - alpha) * imu_1.acc + alpha * imu_2.acc;
	data.gyr = (1 - alpha) * imu_1.gyr + alpha * imu_2.gyr;
	
	return data;
}

std::vector<imuData> selectImuReadings(const std::vector<imuData> &v_imu_data_, double time_0, double time_1, bool warn)
{
	std::vector<imuData> prop_data;

	if (v_imu_data_.empty())
	{
		if (warn)
			std::cout << "selectImuReadings(): No IMU measurements. IMU-CAMERA are likely messed up!" << std::endl;
		return prop_data;
	}

	for (size_t i = 0; i < v_imu_data_.size() - 1; i++)
	{
		if (v_imu_data_.at(i + 1).timestamp > time_0 && v_imu_data_.at(i).timestamp < time_0)
		{
			imuData data = interpolateData(v_imu_data_.at(i), v_imu_data_.at(i + 1), time_0);
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
				imuData data = interpolateData(v_imu_data_.at(i - 1), v_imu_data_.at(i), time_1);
				prop_data.push_back(data);
			}
			else
			{
				prop_data.push_back(v_imu_data_.at(i));
			}
			
			if (prop_data.at(prop_data.size() - 1).timestamp != time_1)
			{
				imuData data = interpolateData(v_imu_data_.at(i), v_imu_data_.at(i + 1), time_1);
				prop_data.push_back(data);
			}
      
			break;
		}
	}

	if (prop_data.empty())
	{
		if (warn)
			std::cout << "selectImuReadings(): No IMU measurements to propagate with (" << (int)prop_data.size() 
				<< " of 2). IMU-CAMERA are likely messed up." << std::endl;
		
		return prop_data;
	}

	if (prop_data.at(prop_data.size() - 1).timestamp != time_1)
	{
		if (warn)
			std::cout << std::fixed << "selectImuReadings(): Missing inertial measurements to propagate with (" 
				<< time_1 - v_imu_data_.at(v_imu_data_.size() - 1).timestamp << " sec missing)!" << std::endl;

		imuData data = interpolateData(v_imu_data_.at(v_imu_data_.size() - 2), v_imu_data_.at(v_imu_data_.size() - 1), time_1);
		prop_data.push_back(data);
	}

	for (size_t i = 0; i < prop_data.size() - 1; i++)
	{
		if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) < 1e-12)
		{
			if (warn)
				std::cout << "selectImuReadings(): Zero DT between IMU reading " 
					<< (int)i << " and " << (int)(i + 1) << ", removing it!" << std::endl;
			
			prop_data.erase(prop_data.begin() + i);
			i--;
		}
	}

	if (prop_data.size() < 2)
	{
		if (warn)
			std::cout << "selectImuReadings(): No IMU measurements to propagate with (" 
				<< (int)prop_data.size() << " of 2). IMU-CAMERA are likely messed up!" << std::endl;

		return prop_data;
	}

	return prop_data;
}