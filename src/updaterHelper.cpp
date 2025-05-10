#include "updaterHelper.h"
#include "state.h"
#include "quatOps.h"

void updaterHelper::getFeatureJacobianRepresentation(std::shared_ptr<state> state_ptr, updaterHelperFeature &feature, Eigen::MatrixXd &H_f,
	std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<baseType>> &x_order)
{
	H_f.resize(3, 3);
	H_f.setIdentity();
	return;
}

void updaterHelper::getFeatureJacobianFull(std::shared_ptr<state> state_ptr, updaterHelperFeature &feature, Eigen::MatrixXd &H_f, 
	Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<baseType>> &x_order)
{
	int total_meas = 0;
	for (auto const &pair : feature.timestamps)
	{
		total_meas += (int)pair.second.size();
	}

	int total_hx = 0;
	std::unordered_map<std::shared_ptr<baseType>, size_t> map_hx;
	for (auto const &pair : feature.timestamps)
	{
		std::shared_ptr<poseJpl> calibration = state_ptr->calib_cam_imu.at(pair.first);
		std::shared_ptr<vec> distortion = state_ptr->cam_intrinsics.at(pair.first);

		if (state_ptr->options.do_calib_camera_pose)
		{
			map_hx.insert({calibration, total_hx});
			x_order.push_back(calibration);
			total_hx += calibration->getSize();
		}

		if (state_ptr->options.do_calib_camera_intrinsics)
		{
			map_hx.insert({distortion, total_hx});
			x_order.push_back(distortion);
			total_hx += distortion->getSize();
		}

		for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++)
		{
			std::shared_ptr<poseJpl> clone_Ci = state_ptr->clones_imu.at(feature.timestamps[pair.first].at(m));

			if (map_hx.find(clone_Ci) == map_hx.end())
			{
				map_hx.insert({clone_Ci, total_hx});
				x_order.push_back(clone_Ci);
				total_hx += clone_Ci->getSize();
			}
		}
	}

	Eigen::Vector3d p_FinG = feature.position_global;
	Eigen::Vector3d p_FinG_fej = feature.position_global_fej;

	int c = 0;
	int jacobsize = 3;

	res = Eigen::VectorXd::Zero(2 * total_meas);
	H_f = Eigen::MatrixXd::Zero(2 * total_meas, jacobsize);
	H_x = Eigen::MatrixXd::Zero(2 * total_meas, total_hx);

	Eigen::MatrixXd dpfg_dlambda;
	std::vector<Eigen::MatrixXd> dpfg_dx;
	std::vector<std::shared_ptr<baseType>> dpfg_dx_order;
	updaterHelper::getFeatureJacobianRepresentation(state_ptr, feature, dpfg_dlambda, dpfg_dx, dpfg_dx_order);

	for (auto const &pair : feature.timestamps)
	{
		std::shared_ptr<vec> distortion = state_ptr->cam_intrinsics.at(pair.first);
		std::shared_ptr<poseJpl> calibration = state_ptr->calib_cam_imu.at(pair.first);
		Eigen::Matrix3d R_ItoC = calibration->getRot();
		Eigen::Vector3d p_IinC = calibration->getPos();

		for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++)
		{
			std::shared_ptr<poseJpl> clone_Ii = state_ptr->clones_imu.at(feature.timestamps[pair.first].at(m));
			Eigen::Matrix3d R_GtoIi = clone_Ii->getRot();
			Eigen::Vector3d p_IiinG = clone_Ii->getPos();

			Eigen::Vector3d p_FinIi = R_GtoIi * (p_FinG - p_IiinG);

			Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;
			Eigen::Vector2d uv_norm;
			uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);

			Eigen::Vector2d uv_dist;
			uv_dist = state_ptr->cam_intrinsics_cameras.at(pair.first)->distortD(uv_norm);

			Eigen::Vector2d uv_m;
			uv_m << (double)feature.uvs[pair.first].at(m)(0), (double)feature.uvs[pair.first].at(m)(1);
			res.block(2 * c, 0, 2, 1) = uv_m - uv_dist;

			if (state_ptr->options.do_fej)
			{
				R_GtoIi = clone_Ii->getRotFej();
				p_IiinG = clone_Ii->getPosFej();

				p_FinIi = R_GtoIi * (p_FinG_fej - p_IiinG);
				p_FinCi = R_ItoC * p_FinIi + p_IinC;
			}

			Eigen::MatrixXd dz_dzn, dz_dzeta;
			state_ptr->cam_intrinsics_cameras.at(pair.first)->computeDistortJacobian(uv_norm, dz_dzn, dz_dzeta);

			Eigen::MatrixXd dzn_dpfc = Eigen::MatrixXd::Zero(2, 3);
			dzn_dpfc << 1 / p_FinCi(2), 0, -p_FinCi(0) / (p_FinCi(2) * p_FinCi(2)), 0, 1 / p_FinCi(2), -p_FinCi(1) / (p_FinCi(2) * p_FinCi(2));

			Eigen::MatrixXd dpfc_dpfg = R_ItoC * R_GtoIi;

			Eigen::MatrixXd dpfc_dclone = Eigen::MatrixXd::Zero(3, 6);
			dpfc_dclone.block(0, 0, 3, 3).noalias() = R_ItoC * quatType::skewSymmetric(p_FinIi);
			dpfc_dclone.block(0, 3, 3, 3) = - dpfc_dpfg;

			Eigen::MatrixXd dz_dpfc = dz_dzn * dzn_dpfc;
			Eigen::MatrixXd dz_dpfg = dz_dpfc * dpfc_dpfg;

			H_f.block(2 * c, 0, 2, H_f.cols()).noalias() = dz_dpfg * dpfg_dlambda;
			H_x.block(2 * c, map_hx[clone_Ii], 2, clone_Ii->getSize()).noalias() = dz_dpfc * dpfc_dclone;

			for (size_t i = 0; i < dpfg_dx_order.size(); i++)
			{
				H_x.block(2 * c, map_hx[dpfg_dx_order.at(i)], 2, dpfg_dx_order.at(i)->getSize()).noalias() += dz_dpfg * dpfg_dx.at(i);
			}

			if (state_ptr->options.do_calib_camera_pose)
			{
				Eigen::MatrixXd dpfc_dcalib = Eigen::MatrixXd::Zero(3, 6);
				dpfc_dcalib.block(0, 0, 3, 3) = quatType::skewSymmetric(p_FinCi - p_IinC);
				dpfc_dcalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

				H_x.block(2 * c, map_hx[calibration], 2, calibration->getSize()).noalias() += dz_dpfc * dpfc_dcalib;
			}

			if (state_ptr->options.do_calib_camera_intrinsics)
			{
				H_x.block(2 * c, map_hx[distortion], 2, distortion->getSize()) = dz_dzeta;
			}

			if (state_ptr->options.use_huber)
			{
				double loss = res.block(2 * c, 0, 2, 1).norm();
				double hw = loss < setting_huber_th ? 1 : setting_huber_th / loss;

				if (hw < 1) hw = sqrt(hw);

				res.block(2 * c, 0, 2, 1) *= hw;
				H_x.block(2 * c, 0, 2, H_x.cols()) *= hw;
				H_f.block(2 * c, 0, 2, H_f.cols()) *= hw;
			}

			c++;
		}
	}
}

void updaterHelper::nullspaceProjectInplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res)
{
	Eigen::JacobiRotation<double> Ho_GR_temp;
  
	for (int n = 0; n < H_f.cols(); ++n)
	{
		for (int m = (int)H_f.rows() - 1; m > n; m--)
		{
			Ho_GR_temp.makeGivens(H_f(m - 1, n), H_f(m, n));
			(H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, Ho_GR_temp.adjoint());
			(H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, Ho_GR_temp.adjoint());
			(res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, Ho_GR_temp.adjoint());
		}
	}

	H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
	res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

	assert(H_x.rows() == res.rows());
}

void updaterHelper::measurementCompressInplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res)
{
	if (H_x.rows() <= H_x.cols())
		return;

	Eigen::JacobiRotation<double> Ho_GR_temp;
	for (int n = 0; n < H_x.cols(); n++)
	{
		for (int m = (int)H_x.rows() - 1; m > n; m--)
		{
			Ho_GR_temp.makeGivens(H_x(m - 1, n), H_x(m, n));

			(H_x.block(m - 1, n, 2, H_x.cols() - n)).applyOnTheLeft(0, 1, Ho_GR_temp.adjoint());
			(res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, Ho_GR_temp.adjoint());
		}
	}

	int r = std::min(H_x.rows(), H_x.cols());

	assert(r <= H_x.rows());
	H_x.conservativeResize(r, H_x.cols());
	res.conservativeResize(r, res.cols());
}