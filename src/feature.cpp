#include "feature.h"
#include "state.h"

void feature::cleanOldMeasurements(const std::vector<double> &valid_times)
{
	for (auto const &pair : timestamps)
	{
		assert(timestamps[pair.first].size() == uvs[pair.first].size());
		assert(timestamps[pair.first].size() == uvs_norm[pair.first].size());

		auto it1 = timestamps[pair.first].begin();
		auto it2 = uvs[pair.first].begin();
		auto it3 = uvs_norm[pair.first].begin();
		auto it4 = frames[pair.first].begin();
		auto it5 = color[pair.first].begin();

		while (it1 != timestamps[pair.first].end())
		{
			if (std::find(valid_times.begin(), valid_times.end(), *it1) == valid_times.end())
			{
				it1 = timestamps[pair.first].erase(it1);
				it2 = uvs[pair.first].erase(it2);
				it3 = uvs_norm[pair.first].erase(it3);

				assert((*it4)->v_feat_ptr.at(pair.first).find(feature_id) != (*it4)->v_feat_ptr.at(pair.first).end());

				if ((*it4)->v_feat_ptr.at(pair.first).find(feature_id) != (*it4)->v_feat_ptr.at(pair.first).end())
				{
					(*it4)->v_feat_ptr.at(pair.first).erase(feature_id);
				}

				it4 = frames[pair.first].erase(it4);
				it5 = color[pair.first].erase(it5);
			}
			else
			{
				it1++;
				it2++;
				it3++;
				it4++;
				it5++;
			}
		}
	}
}

void feature::cleanInvalidMeasurements(const std::vector<double> &invalid_times)
{
	for (auto const &pair : timestamps)
	{
		assert(timestamps[pair.first].size() == uvs[pair.first].size());
		assert(timestamps[pair.first].size() == uvs_norm[pair.first].size());

		auto it1 = timestamps[pair.first].begin();
		auto it2 = uvs[pair.first].begin();
		auto it3 = uvs_norm[pair.first].begin();
		auto it4 = frames[pair.first].begin();
		auto it5 = color[pair.first].begin();

		while (it1 != timestamps[pair.first].end())
		{
			if (std::find(invalid_times.begin(), invalid_times.end(), *it1) != invalid_times.end())
			{
				it1 = timestamps[pair.first].erase(it1);
				it2 = uvs[pair.first].erase(it2);
				it3 = uvs_norm[pair.first].erase(it3);

				assert((*it4)->v_feat_ptr.at(pair.first).find(feature_id) != (*it4)->v_feat_ptr.at(pair.first).end());

				if ((*it4)->v_feat_ptr.at(pair.first).find(feature_id) != (*it4)->v_feat_ptr.at(pair.first).end())
				{
					(*it4)->v_feat_ptr.at(pair.first).erase(feature_id);
				}

				it4 = frames[pair.first].erase(it4);
				it5 = color[pair.first].erase(it5);
				// 2024.10.31 yzk
			}
			else
			{
				it1++;
				it2++;
				it3++;
				it4++;
				it5++;
			}
		}
	}
}

void feature::cleanOlderMeasurements(double timestamp)
{
	for (auto const &pair : timestamps)
	{
		assert(timestamps[pair.first].size() == uvs[pair.first].size());
		assert(timestamps[pair.first].size() == uvs_norm[pair.first].size());

		auto it1 = timestamps[pair.first].begin();
		auto it2 = uvs[pair.first].begin();
		auto it3 = uvs_norm[pair.first].begin();
		auto it4 = frames[pair.first].begin();
		auto it5 = color[pair.first].begin();

		while (it1 != timestamps[pair.first].end())
		{
			if (*it1 <= timestamp)
			{
				it1 = timestamps[pair.first].erase(it1);
				it2 = uvs[pair.first].erase(it2);
				it3 = uvs_norm[pair.first].erase(it3);

				assert((*it4)->v_feat_ptr.at(pair.first).find(feature_id) != (*it4)->v_feat_ptr.at(pair.first).end());

				if ((*it4)->v_feat_ptr.at(pair.first).find(feature_id) != (*it4)->v_feat_ptr.at(pair.first).end())
				{
					(*it4)->v_feat_ptr.at(pair.first).erase(feature_id);
				}

				it4 = frames[pair.first].erase(it4);
				it5 = color[pair.first].erase(it5);
			}
			else
			{
				it1++;
				it2++;
				it3++;
				it4++;
				it5++;
			}
		}
	}
}

featureDatabase::featureDatabase()
{
	last_track_num = 0;
}

std::shared_ptr<feature> featureDatabase::getFeature(size_t id, bool remove)
{
	if (features_id_lookup.find(id) != features_id_lookup.end())
	{
		std::shared_ptr<feature> temp = features_id_lookup.at(id);
		if (remove)
		{
			for (int i = 0; i < 2; i++)
			{
				for (auto it_fh = temp->frames[i].begin(); it_fh != temp->frames[i].end(); it_fh++)
				{
					if ((*it_fh) == nullptr || (*it_fh)->is_invalid) continue;

					if ((*it_fh)->v_feat_ptr.at(i).find(temp->feature_id) != (*it_fh)->v_feat_ptr.at(i).end())
						(*it_fh)->v_feat_ptr.at(i).erase(temp->feature_id);
				}
			}

			features_id_lookup.erase(id);
		}
		return temp;
	}
	else
	{
		return nullptr;
	}
}

void featureDatabase::updateFeature(std::shared_ptr<frame> fh, size_t id, double timestamp, size_t cam_id, float u, float v, float u_n, float v_n)
{
	if (features_id_lookup.find(id) != features_id_lookup.end())
	{
		std::shared_ptr<feature> feat = features_id_lookup.at(id);

		if (u > 1.1 && u < wG[0] - 3 && v > 1.1 && v < hG[0] - 3)
		{
			feat->uvs[cam_id].push_back(Eigen::Vector2f(u, v));
			feat->uvs_norm[cam_id].push_back(Eigen::Vector2f(u_n, v_n));
			feat->timestamps[cam_id].push_back(timestamp);
			feat->frames[cam_id].push_back(fh);

			float color_temp = cam_id == 0 ? feat->frames[cam_id].back()->dI_left[(int)u + (int)v * wG[0]][0] 
				: feat->frames[cam_id].back()->dI_right[(int)u + (int)v * wG[0]][0];

			assert(color_temp >= 0);

			feat->color[cam_id].push_back(color_temp);
			feat->num_tracking = fh->frame_id - feat->start_frame_id;
			fh->v_feat_ptr[cam_id][feat->feature_id] = feat;

			last_track_num++;
		}

		return;
	}

	if (u > 1.1 && u < wG[0] - 3 && v > 1.1 && v < hG[0] - 3)
	{
		std::shared_ptr<feature> feat = std::make_shared<feature>();
		feat->feature_id = id;
		feat->uvs[cam_id].push_back(Eigen::Vector2f(u, v));
		feat->uvs_norm[cam_id].push_back(Eigen::Vector2f(u_n, v_n));
		feat->timestamps[cam_id].push_back(timestamp);
		feat->frames[cam_id].push_back(fh);

		float color_temp = cam_id == 0 ? feat->frames[cam_id].back()->dI_left[(int)u + (int)v * wG[0]][0] 
			: feat->frames[cam_id].back()->dI_right[(int)u + (int)v * wG[0]][0];

		assert(color_temp >= 0);

		feat->color[cam_id].push_back(color_temp);
		feat->start_frame_id = fh->frame_id;
		feat->start_cam_id = cam_id;
		feat->num_tracking = 0;

		features_id_lookup[id] = feat;

		fh->v_feat_ptr[cam_id][feat->feature_id] = feat;
	}
}

std::vector<std::shared_ptr<feature>> featureDatabase::getOldFeatures(double timestamp, bool remove, bool skip_deleted)
{
	std::vector<std::shared_ptr<feature>> feats_old;

	for (auto it = features_id_lookup.begin(); it != features_id_lookup.end();)
	{
		// Skip if already deleted
		if (skip_deleted && (*it).second->to_delete)
		{
			it++;
			continue;
		}

		bool has_newer_measurement = false;
		for (auto const &pair : (*it).second->timestamps)
		{
			has_newer_measurement = (!pair.second.empty() && pair.second.at(pair.second.size() - 1) >= timestamp);
			if (has_newer_measurement)
			{
				break;
			}
		}

		if (!has_newer_measurement)
		{
			feats_old.push_back((*it).second);

			if (remove)
			{
				for (int i = 0; i < 2; i++)
				{
					for (auto it_fh = (*it).second->frames[i].begin(); it_fh != (*it).second->frames[i].end(); it_fh++)
					{
						if ((*it_fh) == nullptr || (*it_fh)->is_invalid) continue;

						if ((*it_fh)->v_feat_ptr.at(i).find((*it).second->feature_id) != (*it_fh)->v_feat_ptr.at(i).end())
							(*it_fh)->v_feat_ptr.at(i).erase((*it).second->feature_id);
					}
				}

				features_id_lookup.erase(it++);
			}
			else
				it++;
		}
		else
		{
			it++;
		}
	}

	return feats_old;
}

std::vector<std::shared_ptr<feature>> featureDatabase::getFeatures(double timestamp, bool remove, bool skip_deleted)
{
	std::vector<std::shared_ptr<feature>> feats_has_timestamp;

	for (auto it = features_id_lookup.begin(); it != features_id_lookup.end();)
	{
		if (skip_deleted && (*it).second->to_delete)
		{
			it++;
			continue;
		}
    
		bool has_timestamp = false;
		for (auto const &pair : (*it).second->timestamps)
		{
			has_timestamp = (std::find(pair.second.begin(), pair.second.end(), timestamp) != pair.second.end());
			if (has_timestamp)
			{
				break;
			}
		}

		if (has_timestamp)
		{
			feats_has_timestamp.push_back((*it).second);

			if (remove)
			{
				for (int i = 0; i < 2; i++)
				{
					for (auto it_fh = (*it).second->frames[i].begin(); it_fh != (*it).second->frames[i].end(); it_fh++)
					{
						if ((*it_fh) == nullptr || (*it_fh)->is_invalid) continue;

						if ((*it_fh)->v_feat_ptr.at(i).find((*it).second->feature_id) != (*it_fh)->v_feat_ptr.at(i).end())
							(*it_fh)->v_feat_ptr.at(i).erase((*it).second->feature_id);
					}
				}

				features_id_lookup.erase(it++);
			}
			else
				it++;
		}
		else
		{
			it++;
		}
	}

	return feats_has_timestamp;
}

void featureDatabase::cleanUp()
{
	for (auto it = features_id_lookup.begin(); it != features_id_lookup.end();)
	{
		if ((*it).second->to_delete)
		{
			for (int i = 0; i < 2; i++)
			{
				for (auto it_fh = (*it).second->frames[i].begin(); it_fh != (*it).second->frames[i].end(); it_fh++)
				{
					if ((*it_fh) == nullptr || (*it_fh)->is_invalid) continue;

					if ((*it_fh)->v_feat_ptr.at(i).find((*it).second->feature_id) != (*it_fh)->v_feat_ptr.at(i).end())
						(*it_fh)->v_feat_ptr.at(i).erase((*it).second->feature_id);
				}
			}

			features_id_lookup.erase(it++);
		}
		else
		{
			it++;
		}
	}
}

void featureDatabase::cleanUpOldmeasurements(double timestamp)
{
	for (auto it = features_id_lookup.begin(); it != features_id_lookup.end();)
	{
		(*it).second->cleanOlderMeasurements(timestamp);
		
		int count_meas = 0;

		for (const auto &pair : (*it).second->timestamps)
		{
			count_meas += (int)(pair.second.size());
		}

		if (count_meas < 1)
		{
			for (int i = 0; i < 2; i++)
			{
				for (auto it_fh = (*it).second->frames[i].begin(); it_fh != (*it).second->frames[i].end(); it_fh++)
				{
					if ((*it_fh) == nullptr || (*it_fh)->is_invalid) continue;

					if ((*it_fh)->v_feat_ptr.at(i).find((*it).second->feature_id) != (*it_fh)->v_feat_ptr.at(i).end())
						(*it_fh)->v_feat_ptr.at(i).erase((*it).second->feature_id);
				}
			}

			features_id_lookup.erase(it++);
		}
		else
		{
			it++;
		}
	}
}

size_t featureDatabase::getSize()
{
	return features_id_lookup.size();
}

std::unordered_map<size_t, std::shared_ptr<feature>> featureDatabase::getInternalData()
{
	return features_id_lookup;
}

bool featureDatabase::featureCheckParallax(int frame_count)
{
    double parallax_sum = 0;
    int parallax_num = 0;

    if (frame_count < 2 || last_track_num < 20)
    {
    	last_track_num = 0;
        return true;
    }

    last_track_num = 0;

    for (auto it = features_id_lookup.begin(); it != features_id_lookup.end(); it++)
	{
		if ((*it).second->start_frame_id <= frame_count - 2)
		{
			if ((*it).second->start_cam_id == 0 && (*it).second->start_frame_id + int((*it).second->timestamps.at(0).size()) - 1 >= frame_count)
			{
				parallax_sum += compensatedParallax2((*it).first, 0, frame_count);
            	parallax_num++;
			}
			else if ((*it).second->start_cam_id == 1 && (*it).second->start_frame_id + int((*it).second->timestamps.at(1).size()) - 1 >= frame_count)
			{
				parallax_sum += compensatedParallax2((*it).first, 1, frame_count);
            	parallax_num++;
			}
		}
	}

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        return parallax_sum / parallax_num >= setting_min_parallax;
    }
}

double featureDatabase::compensatedParallax2(int feat_id, int cam_id, int frame_count)
{
	std::shared_ptr<feature> feat_ptr = features_id_lookup.at(feat_id);

	Eigen::Vector2f uvs_norm_i = feat_ptr->uvs_norm.at(cam_id).at(frame_count - 2 - feat_ptr->start_frame_id);
	Eigen::Vector2f uvs_norm_j = feat_ptr->uvs_norm.at(cam_id).at(frame_count - 1 - feat_ptr->start_frame_id);

    float ans = 0;
    float du = uvs_norm_i(0) - uvs_norm_j(0);
    float dv = uvs_norm_i(1) - uvs_norm_j(1);

    ans = std::max(ans, sqrt(du * du + dv * dv));

    return ans;
}



featureInitializer::featureInitializer(featureInitializerOptions &options_) : options(options_)
{

}

bool featureInitializer::singleTriangulation(std::shared_ptr<feature> feat, std::unordered_map<size_t, std::unordered_map<double, clonePose>> &clones_cam)
{
	int total_meas = 0;
	size_t anchor_most_meas = 0;
	size_t most_meas = 0;

	for (auto const &pair : feat->timestamps)
	{	
		total_meas += (int)pair.second.size();
		
		if (pair.second.size() > most_meas)
    	{
			anchor_most_meas = pair.first;
			most_meas = pair.second.size();
		}
	}

	feat->anchor_cam_id = anchor_most_meas;
	feat->anchor_clone_timestamp = feat->timestamps.at(feat->anchor_cam_id).back();

	Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
	Eigen::Vector3d b = Eigen::Vector3d::Zero();

	clonePose anchor_clone = clones_cam.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp);
	const Eigen::Matrix<double, 3, 3> &R_GtoA = anchor_clone.Rot();
	const Eigen::Matrix<double, 3, 1> &p_AinG = anchor_clone.pos();

	for (auto const &pair : feat->timestamps)
	{
		for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++)
		{
			const Eigen::Matrix<double, 3, 3> &R_GtoCi = clones_cam.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).Rot();
			const Eigen::Matrix<double, 3, 1> &p_CiinG = clones_cam.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).pos();

			Eigen::Matrix<double, 3, 3> R_AtoCi;
			R_AtoCi.noalias() = R_GtoCi * R_GtoA.transpose();
			Eigen::Matrix<double, 3, 1> p_CiinA;
			p_CiinA.noalias() = R_GtoA * (p_CiinG - p_AinG);

			Eigen::Matrix<double, 3, 1> b_i;
			b_i << feat->uvs_norm.at(pair.first).at(m)(0), feat->uvs_norm.at(pair.first).at(m)(1), 1;
			b_i = R_AtoCi.transpose() * b_i;
			b_i = b_i / b_i.norm();
			Eigen::Matrix3d B_perp = quatType::skewSymmetric(b_i);

			Eigen::Matrix3d Ai = B_perp.transpose() * B_perp;
			A += Ai;
			b += Ai * p_CiinA;
		}
	}

	Eigen::MatrixXd p_f = A.colPivHouseholderQr().solve(b);

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(A);
	Eigen::MatrixXd singular_values;
	singular_values.resize(svd.singularValues().rows(), 1);
	singular_values = svd.singularValues();
	double cond_A = singular_values(0, 0) / singular_values(singular_values.rows() - 1, 0);

	if (std::abs(cond_A) > options.max_cond_number || p_f(2, 0) < options.min_dist || p_f(2, 0) > options.max_dist || std::isnan(p_f.norm()))
	{
		return false;
	}

	feat->position_anchor = p_f;
	feat->position_global = R_GtoA.transpose() * feat->position_anchor + p_AinG;

	return true;
}

bool featureInitializer::singleGaussnewton(std::shared_ptr<feature> feat, std::unordered_map<size_t, std::unordered_map<double, clonePose>> &clones_cam)
{
	double rho = 1 / feat->position_anchor(2);
	double alpha = feat->position_anchor(0) / feat->position_anchor(2);
	double beta = feat->position_anchor(1) / feat->position_anchor(2);

	double lam = options.init_lamda;
	double eps = 10000;
	int runs = 0;

	bool recompute = true;
	Eigen::Matrix<double, 3, 3> Hess = Eigen::Matrix<double, 3, 3>::Zero();
	Eigen::Matrix<double, 3, 1> grad = Eigen::Matrix<double, 3, 1>::Zero();

	double cost_old = computeError(clones_cam, feat, alpha, beta, rho);

	const Eigen::Matrix<double, 3, 3> &R_GtoA = clones_cam.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).Rot();
	const Eigen::Matrix<double, 3, 1> &p_AinG = clones_cam.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).pos();

	while (runs < options.max_runs && lam < options.max_lamda && eps > options.min_dx)
	{
		if (recompute)
		{
			Hess.setZero();
			grad.setZero();

			double err = 0;

			for (auto const &pair : feat->timestamps)
			{
				for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++)
				{
					const Eigen::Matrix<double, 3, 3> &R_GtoCi = clones_cam.at(pair.first).at(feat->timestamps[pair.first].at(m)).Rot();
					const Eigen::Matrix<double, 3, 1> &p_CiinG = clones_cam.at(pair.first).at(feat->timestamps[pair.first].at(m)).pos();

					Eigen::Matrix<double, 3, 3> R_AtoCi;
					R_AtoCi.noalias() = R_GtoCi * R_GtoA.transpose();
					Eigen::Matrix<double, 3, 1> p_CiinA;
					p_CiinA.noalias() = R_GtoA * (p_CiinG - p_AinG);
					Eigen::Matrix<double, 3, 1> p_AinCi;
					p_AinCi.noalias() = -R_AtoCi * p_CiinA;

					double hi1 = R_AtoCi(0, 0) * alpha + R_AtoCi(0, 1) * beta + R_AtoCi(0, 2) + rho * p_AinCi(0, 0);
					double hi2 = R_AtoCi(1, 0) * alpha + R_AtoCi(1, 1) * beta + R_AtoCi(1, 2) + rho * p_AinCi(1, 0);
					double hi3 = R_AtoCi(2, 0) * alpha + R_AtoCi(2, 1) * beta + R_AtoCi(2, 2) + rho * p_AinCi(2, 0);

					double d_z1_d_alpha = (R_AtoCi(0, 0) * hi3 - hi1 * R_AtoCi(2, 0)) / (pow(hi3, 2));
					double d_z1_d_beta = (R_AtoCi(0, 1) * hi3 - hi1 * R_AtoCi(2, 1)) / (pow(hi3, 2));
					double d_z1_d_rho = (p_AinCi(0, 0) * hi3 - hi1 * p_AinCi(2, 0)) / (pow(hi3, 2));
					double d_z2_d_alpha = (R_AtoCi(1, 0) * hi3 - hi2 * R_AtoCi(2, 0)) / (pow(hi3, 2));
					double d_z2_d_beta = (R_AtoCi(1, 1) * hi3 - hi2 * R_AtoCi(2, 1)) / (pow(hi3, 2));
					double d_z2_d_rho = (p_AinCi(1, 0) * hi3 - hi2 * p_AinCi(2, 0)) / (pow(hi3, 2));
					Eigen::Matrix<double, 2, 3> H;
					H << d_z1_d_alpha, d_z1_d_beta, d_z1_d_rho, d_z2_d_alpha, d_z2_d_beta, d_z2_d_rho;

					Eigen::Matrix<float, 2, 1> z;
					z << hi1 / hi3, hi2 / hi3;
					Eigen::Matrix<float, 2, 1> res = feat->uvs_norm.at(pair.first).at(m) - z;

					err += std::pow(res.norm(), 2);
					grad.noalias() += H.transpose() * res.cast<double>();
					Hess.noalias() += H.transpose() * H;
				}
			}
		}

		Eigen::Matrix<double, 3, 3> Hess_l = Hess;

		for (size_t r = 0; r < (size_t)Hess.rows(); r++)
		{
			Hess_l(r, r) *= (1.0 + lam);
		}

		Eigen::Matrix<double, 3, 1> dx = Hess_l.colPivHouseholderQr().solve(grad);

		double cost = computeError(clones_cam, feat, alpha + dx(0, 0), beta + dx(1, 0), rho + dx(2, 0));

		if (cost <= cost_old && (cost_old - cost) / cost_old < options.min_dcost)
		{
			alpha += dx(0, 0);
			beta += dx(1, 0);
			rho += dx(2, 0);
			eps = 0;
			break;
		}

		if (cost <= cost_old)
		{
			recompute = true;
			cost_old = cost;
			alpha += dx(0, 0);
			beta += dx(1, 0);
			rho += dx(2, 0);
			runs++;
			lam = lam / options.lam_mult;
			eps = dx.norm();
		}
		else
		{
			recompute = false;
			lam = lam * options.lam_mult;
			continue;
		}
	}

	feat->position_anchor(0) = alpha / rho;
	feat->position_anchor(1) = beta / rho;
	feat->position_anchor(2) = 1 / rho;

	Eigen::HouseholderQR<Eigen::MatrixXd> qr(feat->position_anchor);
	Eigen::MatrixXd Q = qr.householderQ();

	double base_line_max = 0.0;

	for (auto const &pair : feat->timestamps)
	{
		for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++)
		{
			const Eigen::Matrix<double, 3, 1> &p_CiinG = clones_cam.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).pos();

			Eigen::Matrix<double, 3, 1> p_CiinA = R_GtoA * (p_CiinG - p_AinG);

			double base_line = ((Q.block(0, 1, 3, 2)).transpose() * p_CiinA).norm();

			if (base_line > base_line_max)
				base_line_max = base_line;
		}
	}

	if (feat->position_anchor(2) < options.min_dist || feat->position_anchor(2) > options.max_dist ||
		(feat->position_anchor.norm() / base_line_max) > options.max_baseline || std::isnan(feat->position_anchor.norm()))
	{
		return false;
	}

	feat->position_global = R_GtoA.transpose() * feat->position_anchor + p_AinG;
	
	return true;
}

double featureInitializer::computeError(std::unordered_map<size_t, std::unordered_map<double, clonePose>> &clones_cam, std::shared_ptr<feature> feat,
	double alpha, double beta, double rho)
{
	double err = 0;

	const Eigen::Matrix<double, 3, 3> &R_GtoA = clones_cam.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).Rot();
	const Eigen::Matrix<double, 3, 1> &p_AinG = clones_cam.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).pos();

	for (auto const &pair : feat->timestamps)
	{
		for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++)
		{
			const Eigen::Matrix<double, 3, 3> &R_GtoCi = clones_cam.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).Rot();
			const Eigen::Matrix<double, 3, 1> &p_CiinG = clones_cam.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).pos();
      
			Eigen::Matrix<double, 3, 3> R_AtoCi;
			R_AtoCi.noalias() = R_GtoCi * R_GtoA.transpose();
			Eigen::Matrix<double, 3, 1> p_CiinA;
			p_CiinA.noalias() = R_GtoA * (p_CiinG - p_AinG);
			Eigen::Matrix<double, 3, 1> p_AinCi;
			p_AinCi.noalias() = -R_AtoCi * p_CiinA;

			double hi1 = R_AtoCi(0, 0) * alpha + R_AtoCi(0, 1) * beta + R_AtoCi(0, 2) + rho * p_AinCi(0, 0);
			double hi2 = R_AtoCi(1, 0) * alpha + R_AtoCi(1, 1) * beta + R_AtoCi(1, 2) + rho * p_AinCi(1, 0);
			double hi3 = R_AtoCi(2, 0) * alpha + R_AtoCi(2, 1) * beta + R_AtoCi(2, 2) + rho * p_AinCi(2, 0);
      
			Eigen::Matrix<float, 2, 1> z;
			z << hi1 / hi3, hi2 / hi3;
			Eigen::Matrix<float, 2, 1> res = feat->uvs_norm.at(pair.first).at(m) - z;

			err += pow(res.norm(), 2);
		}
	}

	return err;
}