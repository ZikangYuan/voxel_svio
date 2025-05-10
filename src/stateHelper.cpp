#include "stateHelper.h"
#include <boost/math/distributions/chi_squared.hpp>

stateHelper::stateHelper()
{

}

void stateHelper::ekfPropagation(std::shared_ptr<state> state_ptr, const std::vector<std::shared_ptr<baseType>> &order_new,
	const std::vector<std::shared_ptr<baseType>> &order_old, const Eigen::MatrixXd &phi, const Eigen::MatrixXd &Q)
{
	if (order_new.empty() || order_old.empty())
	{
		std::cout << "[stateHelper::ekfPropagation]: Called with empty variable arrays!" << std::endl;
		std::exit(EXIT_FAILURE);
	}

	int size_order_new = order_new.at(0)->getSize();
	for (size_t i = 0; i < order_new.size() - 1; i++)
	{
		if (order_new.at(i)->getId() + order_new.at(i)->getSize() != order_new.at(i + 1)->getId())
		{
			std::cout << "[stateHelper::ekfPropagation]: Called with non-contiguous state elements!" << std::endl;
			std::cout << "[stateHelper::ekfPropagation]: This code only support a state transition which is in the same order as the state." << std::endl;
			std::exit(EXIT_FAILURE);
		}
		size_order_new += order_new.at(i + 1)->getSize();
	}

	int size_order_old = order_old.at(0)->getSize();
	for (size_t i = 0; i < order_old.size() - 1; i++)
	{
		size_order_old += order_old.at(i + 1)->getSize();
	}

	assert(size_order_new == phi.rows());
	assert(size_order_old == phi.cols());
	assert(size_order_new == Q.cols());
	assert(size_order_new == Q.rows());

	int current_it = 0;
	std::vector<int> phi_id;
	for (const auto &var : order_old)
	{
		phi_id.push_back(current_it);
		current_it += var->getSize();
	}

	Eigen::MatrixXd cov_phiT = Eigen::MatrixXd::Zero(state_ptr->cov.rows(), phi.rows());
	for (size_t i = 0; i < order_old.size(); i++)
	{
		std::shared_ptr<baseType> var = order_old.at(i);
		cov_phiT.noalias() += state_ptr->cov.block(0, var->getId(), state_ptr->cov.rows(), var->getSize()) * phi.block(0, phi_id[i], phi.rows(), var->getSize()).transpose();
	}

	Eigen::MatrixXd phi_cov_phiT = Q.selfadjointView<Eigen::Upper>();
	for (size_t i = 0; i < order_old.size(); i++)
	{
		std::shared_ptr<baseType> var = order_old.at(i);
		phi_cov_phiT.noalias() += phi.block(0, phi_id[i], phi.rows(), var->getSize()) * cov_phiT.block(var->getId(), 0, var->getSize(), phi.rows());
	}

	int start_id = order_new.at(0)->getId();
	int phi_size = phi.rows();
	int total_size = state_ptr->cov.rows();
	state_ptr->cov.block(start_id, 0, phi_size, total_size) = cov_phiT.transpose();
	state_ptr->cov.block(0, start_id, total_size, phi_size) = cov_phiT;
	state_ptr->cov.block(start_id, start_id, phi_size, phi_size) = phi_cov_phiT;

	Eigen::VectorXd diags = state_ptr->cov.diagonal();
	bool found_neg = false;
	for (int i = 0; i < diags.rows(); i++)
	{
		if (diags(i) < 0.0)
		{
			std::cout << "[stateHelper::ekfPropagation]: Diagonal at " << i << " is " << diags(i) << std::endl;
			found_neg = true;
		}
	}

	if (found_neg)
	{
		std::exit(EXIT_FAILURE);
	}
}

void stateHelper::ekfUpdate(std::shared_ptr<state> state_ptr, const std::vector<std::shared_ptr<baseType>> &H_order, 
	const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R)
{
	assert(res.rows() == R.rows());
	assert(H.rows() == res.rows());
	Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state_ptr->cov.rows(), res.rows());

	int current_it = 0;
	std::vector<int> H_id;
	for (const auto &meas_var : H_order)
	{
		H_id.push_back(current_it);
		current_it += meas_var->getSize();
	}

	for (const auto &var : state_ptr->variables)
	{
		// Sum up effect of each subjacobian = K_i= \sum_m (P_im Hm^T)
		Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->getSize(), res.rows());
		for (size_t i = 0; i < H_order.size(); i++)
		{
			std::shared_ptr<baseType> meas_var = H_order[i];
			M_i.noalias() += state_ptr->cov.block(var->getId(), meas_var->getId(), var->getSize(), meas_var->getSize()) * 
				H.block(0, H_id[i], H.rows(), meas_var->getSize()).transpose();
		}
		M_a.block(var->getId(), 0, var->getSize(), res.rows()) = M_i;
	}

	Eigen::MatrixXd P_small = stateHelper::getMarginalCovariance(state_ptr, H_order);
	Eigen::MatrixXd S = H * P_small * H.transpose() + R;
	Eigen::MatrixXd S_inv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
	S.selfadjointView<Eigen::Upper>().llt().solveInPlace(S_inv);
	Eigen::MatrixXd K = M_a * S_inv.selfadjointView<Eigen::Upper>();

	state_ptr->cov.triangularView<Eigen::Upper>() -= K * M_a.transpose();
	state_ptr->cov = state_ptr->cov.selfadjointView<Eigen::Upper>();

	Eigen::VectorXd diags = state_ptr->cov.diagonal();
	bool found_neg = false;

	for (int i = 0; i < diags.rows(); i++)
	{
		if (diags(i) < 0.0)
		{
			std::cout << "[stateHelper::ekfUpdate: Diagonal at " << i << " is " << diags(i) << std::endl;
			found_neg = true;
		}
	}

	if (found_neg)
	{
		std::exit(EXIT_FAILURE);
	}

	Eigen::VectorXd dx = K * res;
	for (size_t i = 0; i < state_ptr->variables.size(); i++)
	{
		state_ptr->variables.at(i)->update(dx.block(state_ptr->variables.at(i)->getId(), 0, state_ptr->variables.at(i)->getSize(), 1));
	}

	if (state_ptr->options.do_calib_camera_intrinsics)
	{
		for (auto const &calib : state_ptr->cam_intrinsics)
		{
			state_ptr->cam_intrinsics_cameras.at(calib.first)->setValue(calib.second->value());
		}
	}
}

void stateHelper::setInitialCovariance(std::shared_ptr<state> state_ptr, const Eigen::MatrixXd &covariance, 
	const std::vector<std::shared_ptr<baseType>> &order)
{
	int i_index = 0;

	for (size_t i = 0; i < order.size(); i++)
	{
		int k_index = 0;
		for (size_t k = 0; k < order.size(); k++)
		{
			state_ptr->cov.block(order[i]->getId(), order[k]->getId(), order[i]->getSize(), order[k]->getSize()) = 
				covariance.block(i_index, k_index, order[i]->getSize(), order[k]->getSize());
			k_index += order[k]->getSize();
		}
		i_index += order[i]->getSize();
	}
	state_ptr->cov = state_ptr->cov.selfadjointView<Eigen::Upper>();
}

Eigen::MatrixXd stateHelper::getMarginalCovariance(std::shared_ptr<state> state_ptr, const std::vector<std::shared_ptr<baseType>> &small_variables)
{
	int cov_size = 0;
	for (size_t i = 0; i < small_variables.size(); i++)
	{
		cov_size += small_variables[i]->getSize();
	}

	Eigen::MatrixXd small_cov = Eigen::MatrixXd::Zero(cov_size, cov_size);
	int i_index = 0;

	for (size_t i = 0; i < small_variables.size(); i++)
	{
		int k_index = 0;

		for (size_t k = 0; k < small_variables.size(); k++)
		{
			small_cov.block(i_index, k_index, small_variables[i]->getSize(), small_variables[k]->getSize()) = 
				state_ptr->cov.block(small_variables[i]->getId(), small_variables[k]->getId(), small_variables[i]->getSize(), small_variables[k]->getSize());
			k_index += small_variables[k]->getSize();
		}

		i_index += small_variables[i]->getSize();
	}

	return small_cov;
}

Eigen::MatrixXd stateHelper::getFullCovariance(std::shared_ptr<state> state_ptr)
{
	int cov_size = (int)state_ptr->cov.rows();

	Eigen::MatrixXd full_cov = Eigen::MatrixXd::Zero(cov_size, cov_size);

	full_cov.block(0, 0, state_ptr->cov.rows(), state_ptr->cov.rows()) = state_ptr->cov;

	return full_cov;
}

void stateHelper::marginalize(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> marg)
{
	if (std::find(state_ptr->variables.begin(), state_ptr->variables.end(), marg) == state_ptr->variables.end())
	{
		std::cout << "[stateHelper::marginalize]: Called on variable that is not in the state." << std::endl;
		std::cout << "[stateHelper::marginalize]: Marginalization, does NOT work on sub-variables yet..." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	int marg_size = marg->getSize();
	int marg_id = marg->getId();
	int x2_size = (int)state_ptr->cov.rows() - marg_id - marg_size;

	Eigen::MatrixXd cov_new(state_ptr->cov.rows() - marg_size, state_ptr->cov.rows() - marg_size);

	cov_new.block(0, 0, marg_id, marg_id) = state_ptr->cov.block(0, 0, marg_id, marg_id);
	cov_new.block(0, marg_id, marg_id, x2_size) = state_ptr->cov.block(0, marg_id + marg_size, marg_id, x2_size);
	cov_new.block(marg_id, 0, x2_size, marg_id) = cov_new.block(0, marg_id, marg_id, x2_size).transpose();
	cov_new.block(marg_id, marg_id, x2_size, x2_size) = state_ptr->cov.block(marg_id + marg_size, marg_id + marg_size, x2_size, x2_size);

	state_ptr->cov = cov_new;
	assert(state_ptr->cov.rows() == cov_new.rows());

	std::vector<std::shared_ptr<baseType>> remaining_variables;

	for (size_t i = 0; i < state_ptr->variables.size(); i++)
	{
		if (state_ptr->variables.at(i) != marg)
		{
			if (state_ptr->variables.at(i)->getId() > marg_id)
			{
				state_ptr->variables.at(i)->setLocalId(state_ptr->variables.at(i)->getId() - marg_size);
			}
			remaining_variables.push_back(state_ptr->variables.at(i));
		}
	}

	marg->setLocalId(-1);
	state_ptr->variables = remaining_variables;
}

std::shared_ptr<baseType> stateHelper::clone(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> variable_to_clone)
{
	int total_size = variable_to_clone->getSize();
	int old_size = (int)state_ptr->cov.rows();
	int new_loc = (int)state_ptr->cov.rows();

	state_ptr->cov.conservativeResizeLike(Eigen::MatrixXd::Zero(old_size + total_size, old_size + total_size));

	const std::vector<std::shared_ptr<baseType>> new_variables = state_ptr->variables;
	std::shared_ptr<baseType> new_clone = nullptr;

	for (size_t k = 0; k < state_ptr->variables.size(); k++)
	{
		std::shared_ptr<baseType> type_check = state_ptr->variables.at(k)->checkIfSubvariable(variable_to_clone);
		if (state_ptr->variables.at(k) == variable_to_clone)
		{
			type_check = state_ptr->variables.at(k);
		}
		else if (type_check != variable_to_clone)
		{
			continue;
		}

		int old_loc = type_check->getId();

		state_ptr->cov.block(new_loc, new_loc, total_size, total_size) = state_ptr->cov.block(old_loc, old_loc, total_size, total_size);
		state_ptr->cov.block(0, new_loc, old_size, total_size) = state_ptr->cov.block(0, old_loc, old_size, total_size);
		state_ptr->cov.block(new_loc, 0, total_size, old_size) = state_ptr->cov.block(old_loc, 0, total_size, old_size);

		new_clone = type_check->clone();
		new_clone->setLocalId(new_loc);
		break;
	}

	if (new_clone == nullptr)
	{
		std::cout << "[stateHelper::clone]: Called on variable is not in the state." << std::endl;
		std::cout << "[stateHelper::clone]: Ensure that the variable specified is a variable, or sub-variable." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	state_ptr->variables.push_back(new_clone);
	return new_clone;
}

bool stateHelper::initialize(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> new_variable, 
	const std::vector<std::shared_ptr<baseType>> &H_order, Eigen::MatrixXd &H_R, Eigen::MatrixXd &H_L, 
	Eigen::MatrixXd &R, Eigen::VectorXd &res, double chi_2_mult)
{
	if (std::find(state_ptr->variables.begin(), state_ptr->variables.end(), new_variable) != state_ptr->variables.end())
	{
		std::cout << "[stateHelper::initializeInvertible]: Called on variable that is already in the state." << std::endl;
		std::cout << "[stateHelper::initializeInvertible]: Found this variable at " << new_variable->getId() << " in covariance." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	assert(R.rows() == R.cols());
	assert(R.rows() > 0);
	for (int r = 0; r < R.rows(); r++)
	{
		for (int c = 0; c < R.cols(); c++)
		{
			if (r == c && R(0, 0) != R(r, c))
			{
				std::cout << "[stateHelper::initialize]: Your noise is not isotropic!" << std::endl;
				std::cout << "[stateHelper::initialize]: Found a value of " << R(r, c) << " verses value of " << R(0, 0) << "." << std::endl;
				std::exit(EXIT_FAILURE);
			}
			else if (r != c && R(r, c) != 0.0)
			{
				std::cout << "[stateHelper::initialize]: Your noise is not diagonal!" << std::endl;
				std::cout << "[stateHelper::initialize]: Found a value of " << R(r, c) << " at row " << r << " and column " << c <<"." << std::endl;
				std::exit(EXIT_FAILURE);
			}
		}
	}

	size_t new_var_size = new_variable->getSize();
	assert((int)new_var_size == H_L.cols());

	Eigen::JacobiRotation<double> tempHo_GR;
	for (int n = 0; n < H_L.cols(); ++n)
	{
		for (int m = (int)H_L.rows() - 1; m > n; m--)
		{
			tempHo_GR.makeGivens(H_L(m - 1, n), H_L(m, n));
			(H_L.block(m - 1, n, 2, H_L.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
			(res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
			(H_R.block(m - 1, 0, 2, H_R.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
		}
	}

	Eigen::MatrixXd Hxinit = H_R.block(0, 0, new_var_size, H_R.cols());
	Eigen::MatrixXd H_finit = H_L.block(0, 0, new_var_size, new_var_size);
	Eigen::VectorXd resinit = res.block(0, 0, new_var_size, 1);
	Eigen::MatrixXd Rinit = R.block(0, 0, new_var_size, new_var_size);

	Eigen::MatrixXd Hup = H_R.block(new_var_size, 0, H_R.rows() - new_var_size, H_R.cols());
	Eigen::VectorXd resup = res.block(new_var_size, 0, res.rows() - new_var_size, 1);
	Eigen::MatrixXd Rup = R.block(new_var_size, new_var_size, R.rows() - new_var_size, R.rows() - new_var_size);

	Eigen::MatrixXd P_up = stateHelper::getMarginalCovariance(state_ptr, H_order);
	assert(Rup.rows() == Hup.rows());
	assert(Hup.cols() == P_up.cols());
	Eigen::MatrixXd S = Hup * P_up * Hup.transpose() + Rup;
	double chi2 = resup.dot(S.llt().solve(resup));

	boost::math::chi_squared chi_squared_dist(res.rows());
	double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
	if (chi2 > chi_2_mult * chi2_check)
	{
		return false;
	}

	stateHelper::initializeInvertible(state_ptr, new_variable, H_order, Hxinit, H_finit, Rinit, resinit);

	if (Hup.rows() > 0)
	{
		stateHelper::ekfUpdate(state_ptr, H_order, Hup, resup, Rup);
	}
	return true;
}

void stateHelper::initializeInvertible(std::shared_ptr<state> state_ptr, std::shared_ptr<baseType> new_variable, 
	const std::vector<std::shared_ptr<baseType>> &H_order, const Eigen::MatrixXd &H_R, 
	const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res)
{
	if (std::find(state_ptr->variables.begin(), state_ptr->variables.end(), new_variable) != state_ptr->variables.end())
	{
		std::cout << "[stateHelper::initializeInvertible]: Called on variable that is already in the state." << std::endl;
		std::cout << "[stateHelper::initializeInvertible]: Found this variable at " << new_variable->getId() << " in covariance." << std::endl;
		std::exit(EXIT_FAILURE);
	}

	assert(R.rows() == R.cols());
	assert(R.rows() > 0);
	for (int r = 0; r < R.rows(); r++)
	{
		for (int c = 0; c < R.cols(); c++)
		{
			if (r == c && R(0, 0) != R(r, c))
			{
				std::cout << "[stateHelper::initializeInvertible]: Your noise is not isotropic!" << std::endl;
				std::cout << "[stateHelper::initializeInvertible]: Found a value of " << R(r, c) << " verses value of " << R(0, 0) << "." << std::endl;
				std::exit(EXIT_FAILURE);
			}
			else if (r != c && R(r, c) != 0.0)
			{
				std::cout << "[stateHelper::initializeInvertible]: Your noise is not diagonal!" << std::endl;
				std::cout << "[stateHelper::initializeInvertible]: Found a value of " << R(r, c) << " at row " << r << " and column " << c << "." << std::endl;
				std::exit(EXIT_FAILURE);
			}
		}
	}

	assert(res.rows() == R.rows());
	assert(H_L.rows() == res.rows());
	assert(H_L.rows() == H_R.rows());
	Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state_ptr->cov.rows(), res.rows());

	int current_it = 0;
	std::vector<int> H_id;
	for (const auto &meas_var : H_order) {
		H_id.push_back(current_it);
		current_it += meas_var->getSize();
	}

	for (const auto &var : state_ptr->variables)
	{
		Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->getSize(), res.rows());
		for (size_t i = 0; i < H_order.size(); i++)
		{
			std::shared_ptr<baseType> meas_var = H_order.at(i);
			M_i += state_ptr->cov.block(var->getId(), meas_var->getId(), var->getSize(), meas_var->getSize()) * 
				H_R.block(0, H_id[i], H_R.rows(), meas_var->getSize()).transpose();
		}
		M_a.block(var->getId(), 0, var->getSize(), res.rows()) = M_i;
	}

	Eigen::MatrixXd P_small = stateHelper::getMarginalCovariance(state_ptr, H_order);

	Eigen::MatrixXd M(H_R.rows(), H_R.rows());
	M.triangularView<Eigen::Upper>() = H_R * P_small * H_R.transpose();
	M.triangularView<Eigen::Upper>() += R;

	assert(H_L.rows() == H_L.cols());
	assert(H_L.rows() == new_variable->getSize());
	Eigen::MatrixXd H_Linv = H_L.inverse();
	Eigen::MatrixXd P_LL = H_Linv * M.selfadjointView<Eigen::Upper>() * H_Linv.transpose();

	size_t oldSize = state_ptr->cov.rows();
	state_ptr->cov.conservativeResizeLike(Eigen::MatrixXd::Zero(oldSize + new_variable->getSize(), oldSize + new_variable->getSize()));
	state_ptr->cov.block(0, oldSize, oldSize, new_variable->getSize()).noalias() = -M_a * H_Linv.transpose();
	state_ptr->cov.block(oldSize, 0, new_variable->getSize(), oldSize) = state_ptr->cov.block(0, oldSize, oldSize, new_variable->getSize()).transpose();
	state_ptr->cov.block(oldSize, oldSize, new_variable->getSize(), new_variable->getSize()) = P_LL;

	new_variable->update(H_Linv * res);
	new_variable->setLocalId(oldSize);
	state_ptr->variables.push_back(new_variable);
}

void stateHelper::augmentClone(std::shared_ptr<state> state_ptr, Eigen::Matrix<double, 3, 1> last_gyr)
{
	if (state_ptr->clones_imu.find(state_ptr->timestamp) != state_ptr->clones_imu.end())
	{
		std::exit(EXIT_FAILURE);
	}

	std::shared_ptr<baseType> pose_temp = stateHelper::clone(state_ptr, state_ptr->imu_ptr->pose());

	std::shared_ptr<poseJpl> pose = std::dynamic_pointer_cast<poseJpl>(pose_temp);
	if (pose == nullptr)
	{
		std::exit(EXIT_FAILURE);
	}

	state_ptr->clones_imu[state_ptr->timestamp] = pose;

	if (state_ptr->options.do_calib_camera_timeoffset)
	{
		Eigen::Matrix<double, 6, 1> dnc_dt = Eigen::MatrixXd::Zero(6, 1);
		dnc_dt.block(0, 0, 3, 1) = last_gyr;
		dnc_dt.block(3, 0, 3, 1) = state_ptr->imu_ptr->getVel();

		state_ptr->cov.block(0, pose->getId(), state_ptr->cov.rows(), 6) += 
			state_ptr->cov.block(0, state_ptr->calib_dt_imu_cam->getId(), state_ptr->cov.rows(), 1) * dnc_dt.transpose();
		state_ptr->cov.block(pose->getId(), 0, 6, state_ptr->cov.rows()) += 
			dnc_dt * state_ptr->cov.block(state_ptr->calib_dt_imu_cam->getId(), 0, 1, state_ptr->cov.rows());
	}
}

void stateHelper::marginalizeOldClone(std::shared_ptr<state> state_ptr)
{
	if ((int)state_ptr->clones_imu.size() > state_ptr->options.max_clone_size)
	{
		double marginal_time = state_ptr->margtimestep();

		assert(marginal_time != INFINITY);
		stateHelper::marginalize(state_ptr, state_ptr->clones_imu.at(marginal_time));

		state_ptr->clones_imu.erase(marginal_time);

		assert(state_ptr->clones_frame.at(marginal_time)->v_feat_ptr[0].size() == 0);
		assert(state_ptr->clones_frame.at(marginal_time)->v_feat_ptr[1].size() == 0);
		state_ptr->clones_frame.at(marginal_time)->release();
		state_ptr->clones_frame.erase(marginal_time);
	}
}

void stateHelper::marginalizeNewClone(std::shared_ptr<state> state_ptr)
{
	if ((int)state_ptr->clones_imu.size() > state_ptr->options.max_clone_size)
	{
		std::vector<double> clone_times;
		for (const auto &clone_imu : state_ptr->clones_imu)
		{
			clone_times.emplace_back(clone_imu.first);
		}

		double marginal_time = clone_times.at(clone_times.size() - 2);

		assert(marginal_time != INFINITY);
		stateHelper::marginalize(state_ptr, state_ptr->clones_imu.at(marginal_time));

		std::shared_ptr<frame> fh_temp = state_ptr->clones_frame.at(marginal_time);

		if (fh_temp->v_feat_ptr.find(0) != fh_temp->v_feat_ptr.end())
		{
			auto it0 = fh_temp->v_feat_ptr.at(0).begin();

			while (it0 != fh_temp->v_feat_ptr.at(0).end())
			{
				std::vector<double> marginal_timestamps = {marginal_time};
				(*it0).second->cleanInvalidMeasurements(marginal_timestamps);
				it0 = fh_temp->v_feat_ptr.at(0).begin();
			}
		}

		if (fh_temp->v_feat_ptr.find(1) != fh_temp->v_feat_ptr.end())
		{
			auto it1 = fh_temp->v_feat_ptr.at(1).begin();

			while (it1 != fh_temp->v_feat_ptr.at(1).end())
			{
				std::vector<double> marginal_timestamps = {marginal_time};
				(*it1).second->cleanInvalidMeasurements(marginal_timestamps);
				it1 = fh_temp->v_feat_ptr.at(1).begin();
			}
		}

		state_ptr->clones_imu.erase(marginal_time);
		state_ptr->clones_frame.at(marginal_time)->release();
		state_ptr->clones_frame.erase(marginal_time);
	}
}

void stateHelper::marginalizeSlam(std::shared_ptr<state> state_ptr, voxelHashMap &voxel_map, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_history)
{
	// display
	points_history->clear();
	// display

	int ct_marginalized = 0;
	auto it0 = state_ptr->map_points.begin();
	while (it0 != state_ptr->map_points.end())
	{
		if ((*it0).second->should_marg)
		{
			mapManagement::deleteFromVoxel(voxel_map, (*it0).second);
			stateHelper::marginalize(state_ptr, (*it0).second);

			// display
			pcl::PointXYZRGB point_temp;
			point_temp.x = (*it0).second->getPointXYZ(false)[0];
			point_temp.y = (*it0).second->getPointXYZ(false)[1];
			point_temp.z = (*it0).second->getPointXYZ(false)[2];
			point_temp.r = (*it0).second->color;
			point_temp.g = (*it0).second->color;
			point_temp.b = (*it0).second->color;
			points_history->points.push_back(point_temp);
			// display

			it0 = state_ptr->map_points.erase(it0);
			ct_marginalized++;
		}
		else
		{
			it0++;
		}
	}
}