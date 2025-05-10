#include "featureHelper.h"

featureHelper::featureHelper()
{

}

void featureHelper::computeDisparity(std::shared_ptr<featureDatabase> db, double time_0, double time_1, double &disp_mean, 
	double &disp_var, int &total_feats)
{
	std::vector<std::shared_ptr<feature>> feats_0 = db->getFeatures(time_0, false, true);

	std::vector<double> disparities;
    
	for (auto &feat : feats_0)
	{
		for (auto &cam_pairs : feat->timestamps)
		{
			size_t cam_id = cam_pairs.first;
			auto it_0 = std::find(feat->timestamps.at(cam_id).begin(), feat->timestamps.at(cam_id).end(), time_0);
			auto it_1 = std::find(feat->timestamps.at(cam_id).begin(), feat->timestamps.at(cam_id).end(), time_1);
      
			if (it_0 == feat->timestamps.at(cam_id).end() || it_1 == feat->timestamps.at(cam_id).end())
				continue;

			auto idx_0 = std::distance(feat->timestamps.at(cam_id).begin(), it_0);
			auto idx_1 = std::distance(feat->timestamps.at(cam_id).begin(), it_1);

			Eigen::Vector2f uv_0 = feat->uvs.at(cam_id).at(idx_0).block(0, 0, 2, 1);
			Eigen::Vector2f uv_1 = feat->uvs.at(cam_id).at(idx_1).block(0, 0, 2, 1);
			disparities.push_back((uv_1 - uv_0).norm());
		}
	}

	if (disparities.size() < 2)
	{
		disp_mean = -1;
		disp_var = -1;
		total_feats = 0;
	}

	disp_mean = 0;
	for (double disp_i : disparities)
	{
		disp_mean += disp_i;
	}
	disp_mean /= (double)disparities.size();
    
	disp_var = 0;
	for (double &disp_i : disparities)
	{
		disp_var += std::pow(disp_i - disp_mean, 2);
	}
	disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
	total_feats = (int)disparities.size();
}

void featureHelper::computeDisparity(std::shared_ptr<featureDatabase> db, double &disp_mean, double &disp_var, int &total_feats,
	double newest_time, double oldest_time)
{
	std::vector<double> disparities;
	for (auto &feat : db->getInternalData())
	{
		for (auto &cam_pairs : feat.second->timestamps)
		{
			if (cam_pairs.second.size() < 2)
				continue;

			size_t cam_id = cam_pairs.first;
			bool found_0 = false;
			bool found_1 = false;
        
			Eigen::Vector2f uv_0 = Eigen::Vector2f::Zero();
			Eigen::Vector2f uv_1 = Eigen::Vector2f::Zero();

			for (size_t idx = 0; idx < feat.second->timestamps.at(cam_id).size(); idx++)
			{
				double time = feat.second->timestamps.at(cam_id).at(idx);
				if ((oldest_time == -1 || time > oldest_time) && !found_0)
				{
					uv_0 = feat.second->uvs.at(cam_id).at(idx).block(0, 0, 2, 1);
					found_0 = true;
					continue;
				}
				if ((newest_time == -1 || time < newest_time) && found_0)
				{
					uv_1 = feat.second->uvs.at(cam_id).at(idx).block(0, 0, 2, 1);
					found_1 = true;
					continue;
				}
			}

			if (!found_0 || !found_1)
				continue;
        
			disparities.push_back((uv_1 - uv_0).norm());
		}
	}

	if (disparities.size() < 2)
	{
		disp_mean = -1;
		disp_var = -1;
		total_feats = 0;
	}

	disp_mean = 0;
	for (double disp_i : disparities)
	{
		disp_mean += disp_i;
	}
    
	disp_mean /= (double)disparities.size();
	disp_var = 0;
	for (double &disp_i : disparities)
	{
		disp_var += std::pow(disp_i - disp_mean, 2);
	}

	disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
	total_feats = (int)disparities.size();
}