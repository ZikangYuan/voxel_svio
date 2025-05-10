#include "preIntegration.h"

preIntegrationBase::preIntegrationBase(double sigma_w_, double sigma_wb_, double sigma_a_, double sigma_ab_, bool imu_avg_)
{
	Q_c.block(0, 0, 3, 3) = std::pow(sigma_w_, 2) * Eigen::Matrix<double, 3, 3>::Identity();
	Q_c.block(3, 3, 3, 3) = std::pow(sigma_wb_, 2) * Eigen::Matrix<double, 3, 3>::Identity();
	Q_c.block(6, 6, 3, 3) = std::pow(sigma_a_, 2) * Eigen::Matrix<double, 3, 3>::Identity();
	Q_c.block(9, 9, 3, 3) = std::pow(sigma_ab_, 2) * Eigen::Matrix<double, 3, 3>::Identity();

	imu_avg = imu_avg_;

	e_1 << 1, 0, 0;
	e_2 << 0, 1, 0;
	e_3 << 0, 0, 1;
	e_1x = quatType::skewSymmetric(e_1);
	e_2x = quatType::skewSymmetric(e_2);
	e_3x = quatType::skewSymmetric(e_3);
}

void preIntegrationBase::setLinearizationPoints(Eigen::Matrix<double, 3, 1> b_w_lin_, Eigen::Matrix<double, 3, 1> b_a_lin_,
	Eigen::Matrix<double, 4, 1> q_k_lin_, Eigen::Matrix<double, 3, 1> grav_)
{
	b_w_lin = b_w_lin_;
	b_a_lin = b_a_lin_;
	q_k_lin = q_k_lin_;
	grav = grav_;
}



void preIntegrationV1::feedImu(double t_0, double t_1, Eigen::Matrix<double, 3, 1> w_m_0, Eigen::Matrix<double, 3, 1> a_m_0, 
	Eigen::Matrix<double, 3, 1> w_m_1, Eigen::Matrix<double, 3, 1> a_m_1)
{
	double delta_t = t_1 - t_0;
	dt += delta_t;

	if (delta_t == 0) return;

	Eigen::Matrix<double, 3, 1> w_hat = w_m_0 - b_w_lin;
	Eigen::Matrix<double, 3, 1> a_hat = a_m_0 - b_a_lin;

	if (imu_avg)
	{
		w_hat += w_m_1 - b_w_lin;
		w_hat = 0.5 * w_hat;
		a_hat += a_m_1 - b_a_lin;
		a_hat = .5 * a_hat;
	}

	Eigen::Matrix<double, 3, 1> w_hat_dt = w_hat * delta_t;

	double w_1 = w_hat(0, 0);
	double w_2 = w_hat(1, 0);
	double w_3 = w_hat(2, 0);

	double mag_w = w_hat.norm();
	double w_dt = mag_w * delta_t;

	bool small_w = (mag_w < 0.008726646);

	double dt_2 = std::pow(delta_t, 2);
	double cos_wt = std::cos(w_dt);
	double sin_wt = std::sin(w_dt);

	Eigen::Matrix<double, 3, 3> w_x = quatType::skewSymmetric(w_hat);
	Eigen::Matrix<double, 3, 3> a_x = quatType::skewSymmetric(a_hat);
	Eigen::Matrix<double, 3, 3> w_tx = quatType::skewSymmetric(w_hat_dt);
	Eigen::Matrix<double, 3, 3> w_x_2 = w_x * w_x;

	Eigen::Matrix<double, 3, 3> R_tau1_tau2 = small_w ? Eigen::Matrix<double, 3, 3>::Identity() - delta_t * w_x + (std::pow(delta_t, 2) / 2) * w_x_2 
		: Eigen::Matrix<double, 3, 3>::Identity() - (sin_wt / mag_w) * w_x + ((1.0 - cos_wt) / (std::pow(mag_w, 2.0))) * w_x_2;

	Eigen::Matrix<double, 3, 3> R_tau1_k = R_tau1_tau2 * R_tau_k;
	Eigen::Matrix<double, 3, 3> R_k_tau1 = R_tau1_k.transpose();

	double f_1;
	double f_2;
	double f_3;
	double f_4;

	if (small_w)
	{
		f_1 = - (std::pow(delta_t, 3) / 3);
		f_2 = (std::pow(delta_t, 4) / 8);
		f_3 = - (std::pow(delta_t, 2) / 2);
		f_4 = (std::pow(delta_t, 3) / 6);
	}
	else
	{
		f_1 = (w_dt * cos_wt - sin_wt) / (std::pow(mag_w, 3));
		f_2 = (std::pow(w_dt, 2) - 2 * cos_wt - 2 * w_dt * sin_wt + 2) / (2 * std::pow(mag_w, 4));
		f_3 = -(1 - cos_wt) / std::pow(mag_w, 2);
		f_4 = (w_dt - sin_wt) / std::pow(mag_w, 3);
	}

	Eigen::Matrix<double, 3, 3> alpha_arg = ((dt_2 / 2.0) * Eigen::Matrix<double, 3, 3>::Identity() + f_1 * w_x + f_2 * w_x_2);
	Eigen::Matrix<double, 3, 3> Beta_arg = (delta_t * Eigen::Matrix<double, 3, 3>::Identity() + f_3 * w_x + f_4 * w_x_2);

	Eigen::MatrixXd H_al = R_k_tau1 * alpha_arg;
	Eigen::MatrixXd H_be = R_k_tau1 * Beta_arg;

	alpha_tau += beta_tau * delta_t + H_al * a_hat;
	beta_tau += H_be * a_hat;

	Eigen::Matrix<double, 3, 3> J_r_tau1 = small_w ? Eigen::Matrix<double, 3, 3>::Identity() - 0.5 * w_tx + (1.0 / 6.0) * w_tx * w_tx 
		: Eigen::Matrix<double, 3, 3>::Identity() - ((1 - cos_wt) / (std::pow((w_dt), 2.0))) * w_tx + ((w_dt - sin_wt) / (std::pow(w_dt, 3.0))) * w_tx * w_tx;

	J_q = R_tau1_tau2 * J_q + J_r_tau1 * delta_t;

	H_a -= H_al;
	H_a += delta_t * H_b;
	H_b -= H_be;

	Eigen::MatrixXd d_R_bw_1 = - R_k_tau1 * quatType::skewSymmetric(J_q * e_1);
	Eigen::MatrixXd d_R_bw_2 = - R_k_tau1 * quatType::skewSymmetric(J_q * e_2);
	Eigen::MatrixXd d_R_bw_3 = - R_k_tau1 * quatType::skewSymmetric(J_q * e_3);

	double df_1_dbw_1;
	double df_1_dbw_2;
	double df_1_dbw_3;

	double df_2_dbw_1;
	double df_2_dbw_2;
	double df_2_dbw_3;

	double df_3_dbw_1;
	double df_3_dbw_2;
	double df_3_dbw_3;

	double df_4_dbw_1;
	double df_4_dbw_2;
	double df_4_dbw_3;

	if (small_w)
	{
		double df_1_dw_mag = - (std::pow(delta_t, 5) / 15);
		df_1_dbw_1 = w_1 * df_1_dw_mag;
		df_1_dbw_2 = w_2 * df_1_dw_mag;
		df_1_dbw_3 = w_3 * df_1_dw_mag;

		double df_2_dw_mag = (std::pow(delta_t, 6) / 72);
		df_2_dbw_1 = w_1 * df_2_dw_mag;
		df_2_dbw_2 = w_2 * df_2_dw_mag;
		df_2_dbw_3 = w_3 * df_2_dw_mag;

		double df_3_dw_mag = - (std::pow(delta_t, 4) / 12);
		df_3_dbw_1 = w_1 * df_3_dw_mag;
		df_3_dbw_2 = w_2 * df_3_dw_mag;
		df_3_dbw_3 = w_3 * df_3_dw_mag;

		double df_4_dw_mag = (std::pow(delta_t, 5) / 60);
		df_4_dbw_1 = w_1 * df_4_dw_mag;
		df_4_dbw_2 = w_2 * df_4_dw_mag;
		df_4_dbw_3 = w_3 * df_4_dw_mag;
	}
	else
	{
		double df_1_dw_mag = (std::pow(w_dt, 2) * sin_wt - 3 * sin_wt + 3 * w_dt * cos_wt) / std::pow(mag_w, 5);
		df_1_dbw_1 = w_1 * df_1_dw_mag;
		df_1_dbw_2 = w_2 * df_1_dw_mag;
		df_1_dbw_3 = w_3 * df_1_dw_mag;

		double df_2_dw_mag = (std::pow(w_dt, 2) - 4 * cos_wt - 4 * w_dt * sin_wt + std::pow(w_dt, 2) * cos_wt + 4) / (std::pow(mag_w, 6));
		df_2_dbw_1 = w_1 * df_2_dw_mag;
		df_2_dbw_2 = w_2 * df_2_dw_mag;
		df_2_dbw_3 = w_3 * df_2_dw_mag;

		double df_3_dw_mag = (2 * (cos_wt - 1) + w_dt * sin_wt) / (std::pow(mag_w, 4));
		df_3_dbw_1 = w_1 * df_3_dw_mag;
		df_3_dbw_2 = w_2 * df_3_dw_mag;
		df_3_dbw_3 = w_3 * df_3_dw_mag;

		double df_4_dw_mag = (2 * w_dt + w_dt * cos_wt - 3 * sin_wt) / (std::pow(mag_w, 5));
		df_4_dbw_1 = w_1 * df_4_dw_mag;
		df_4_dbw_2 = w_2 * df_4_dw_mag;
		df_4_dbw_3 = w_3 * df_4_dw_mag;
	}

	J_a += J_b * delta_t;
	J_a.block(0, 0, 3, 1) +=
		(d_R_bw_1 * alpha_arg + R_k_tau1 * (df_1_dbw_1 * w_x - f_1 * e_1x + df_2_dbw_1 * w_x_2 - f_2 * (e_1x * w_x + w_x * e_1x))) * a_hat;
	J_a.block(0, 1, 3, 1) +=
		(d_R_bw_2 * alpha_arg + R_k_tau1 * (df_1_dbw_2 * w_x - f_1 * e_2x + df_2_dbw_2 * w_x_2 - f_2 * (e_2x * w_x + w_x * e_2x))) * a_hat;
	J_a.block(0, 2, 3, 1) +=
		(d_R_bw_3 * alpha_arg + R_k_tau1 * (df_1_dbw_3 * w_x - f_1 * e_3x + df_2_dbw_3 * w_x_2 - f_2 * (e_3x * w_x + w_x * e_3x))) * a_hat;
	J_b.block(0, 0, 3, 1) +=
		(d_R_bw_1 * Beta_arg + R_k_tau1 * (df_3_dbw_1 * w_x - f_3 * e_1x + df_4_dbw_1 * w_x_2 - f_4 * (e_1x * w_x + w_x * e_1x))) * a_hat;
	J_b.block(0, 1, 3, 1) +=
		(d_R_bw_2 * Beta_arg + R_k_tau1 * (df_3_dbw_2 * w_x - f_3 * e_2x + df_4_dbw_2 * w_x_2 - f_4 * (e_2x * w_x + w_x * e_2x))) * a_hat;
	J_b.block(0, 2, 3, 1) +=
		(d_R_bw_3 * Beta_arg + R_k_tau1 * (df_3_dbw_3 * w_x - f_3 * e_3x + df_4_dbw_3 * w_x_2 - f_4 * (e_3x * w_x + w_x * e_3x))) * a_hat;

	Eigen::Matrix<double, 3, 3> R_mid = small_w ? Eigen::Matrix<double, 3, 3>::Identity() - 0.5 * delta_t * w_x + (std::pow(.5 * delta_t, 2) / 2) * w_x_2 
		: Eigen::Matrix<double, 3, 3>::Identity() - (std::sin(mag_w * 0.5 * delta_t) / mag_w) * w_x + ((1.0 - std::cos(mag_w * 0.5 * delta_t)) / (std::pow(mag_w, 2.0))) * w_x_2;
	R_mid = R_mid * R_tau_k;

	Eigen::Matrix<double, 15, 15> F_k1 = Eigen::Matrix<double, 15, 15>::Zero();
	F_k1.block(0, 0, 3, 3) = - w_x;
	F_k1.block(0, 3, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	F_k1.block(6, 0, 3, 3) = - R_tau_k.transpose() * a_x;
	F_k1.block(6, 9, 3, 3) = - R_tau_k.transpose();
	F_k1.block(12, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 15, 12> G_k1 = Eigen::Matrix<double, 15, 12>::Zero();
	G_k1.block(0, 0, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	G_k1.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	G_k1.block(6, 6, 3, 3) = - R_tau_k.transpose();
	G_k1.block(9, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 15, 15> P_dot_k1 = F_k1 * P_meas + P_meas * F_k1.transpose() + G_k1 * Q_c * G_k1.transpose();

	Eigen::Matrix<double, 15, 15> F_k2 = Eigen::Matrix<double, 15, 15>::Zero();
	F_k2.block(0, 0, 3, 3) = - w_x;
	F_k2.block(0, 3, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	F_k2.block(6, 0, 3, 3) = - R_mid.transpose() * a_x;
	F_k2.block(6, 9, 3, 3) = - R_mid.transpose();
	F_k2.block(12, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 15, 12> G_k2 = Eigen::Matrix<double, 15, 12>::Zero();
	G_k2.block(0, 0, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	G_k2.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	G_k2.block(6, 6, 3, 3) = - R_mid.transpose();
	G_k2.block(9, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 15, 15> P_k2 = P_meas + P_dot_k1 * delta_t / 2.0;
	Eigen::Matrix<double, 15, 15> P_dot_k2 = F_k2 * P_k2 + P_k2 * F_k2.transpose() + G_k2 * Q_c * G_k2.transpose();

	Eigen::Matrix<double, 15, 15> F_k3 = F_k2;
	Eigen::Matrix<double, 15, 12> G_k3 = G_k2;

	Eigen::Matrix<double, 15, 15> P_k3 = P_meas + P_dot_k2 * delta_t / 2.0;
	Eigen::Matrix<double, 15, 15> P_dot_k3 = F_k3 * P_k3 + P_k3 * F_k3.transpose() + G_k3 * Q_c * G_k3.transpose();

	Eigen::Matrix<double, 15, 15> F_k4 = Eigen::Matrix<double, 15, 15>::Zero();
	F_k4.block(0, 0, 3, 3) = - w_x;
	F_k4.block(0, 3, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	F_k4.block(6, 0, 3, 3) = - R_tau1_k.transpose() * a_x;
	F_k4.block(6, 9, 3, 3) = - R_tau1_k.transpose();
	F_k4.block(12, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 15, 12> G_k4 = Eigen::Matrix<double, 15, 12>::Zero();
	G_k4.block(0, 0, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	G_k4.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	G_k4.block(6, 6, 3, 3) = - R_tau1_k.transpose();
	G_k4.block(9, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 15, 15> P_k4 = P_meas + P_dot_k3 * delta_t;
	Eigen::Matrix<double, 15, 15> P_dot_k4 = F_k4 * P_k4 + P_k4 * F_k4.transpose() + G_k4 * Q_c * G_k4.transpose();

	P_meas += (delta_t / 6.0) * (P_dot_k1 + 2.0 * P_dot_k2 + 2.0 * P_dot_k3 + P_dot_k4);
	P_meas = 0.5 * (P_meas + P_meas.transpose());

	R_tau_k = R_tau1_k;
	q_tau_k = quatType::rotToQuat(R_tau_k);
}



void preIntegrationV2::feedImu(double t_0, double t_1, Eigen::Matrix<double, 3, 1> w_m_0, Eigen::Matrix<double, 3, 1> a_m_0, 
	Eigen::Matrix<double, 3, 1> w_m_1, Eigen::Matrix<double, 3, 1> a_m_1)
{
	double delta_t = t_1 - t_0;
	dt += delta_t;

	if (delta_t == 0) return;

	Eigen::Matrix<double, 3, 1> w_hat = w_m_0 - b_w_lin;
	Eigen::Matrix<double, 3, 1> a_hat = a_m_0 - b_a_lin - R_tau_k * quatType::quatToRot(q_k_lin) * grav;

	if (imu_avg)
	{
		w_hat += w_m_1 - b_w_lin;
		w_hat = 0.5 * w_hat;
	}

	Eigen::Matrix<double, 3, 1> w_hat_dt = w_hat * delta_t;

	double w_1 = w_hat(0, 0);
	double w_2 = w_hat(1, 0);
	double w_3 = w_hat(2, 0);

	double mag_w = w_hat.norm();
	double w_dt = mag_w * delta_t;

	bool small_w = (mag_w < 0.008726646);

	double dt_2 = std::pow(delta_t, 2);
	double cos_wt = std::cos(w_dt);
	double sin_wt = std::sin(w_dt);

	Eigen::Matrix<double, 3, 3> w_x = quatType::skewSymmetric(w_hat);
	Eigen::Matrix<double, 3, 3> w_tx = quatType::skewSymmetric(w_hat_dt);
	Eigen::Matrix<double, 3, 3> w_x_2 = w_x * w_x;

	Eigen::Matrix<double, 3, 3> R_tau1_tau2 = small_w ? Eigen::Matrix<double, 3, 3>::Identity() - delta_t * w_x + (std::pow(delta_t, 2) / 2) * w_x_2 
		: Eigen::Matrix<double, 3, 3>::Identity() - (sin_wt / mag_w) * w_x + ((1.0 - cos_wt) / (std::pow(mag_w, 2.0))) * w_x_2;

	Eigen::Matrix<double, 3, 3> R_tau1_k = R_tau1_tau2 * R_tau_k;
	Eigen::Matrix<double, 3, 3> R_k_tau1 = R_tau1_k.transpose();

	if (imu_avg)
	{
		a_hat += a_m_1 - b_a_lin - R_tau1_k * quatType::quatToRot(q_k_lin) * grav;
		a_hat = 0.5 * a_hat;
	}
	Eigen::Matrix<double, 3, 3> a_x = quatType::skewSymmetric(a_hat);

	double f_1;
	double f_2;
	double f_3;
	double f_4;

	if (small_w)
	{
		f_1 = - (std::pow(delta_t, 3) / 3);
		f_2 = (std::pow(delta_t, 4) / 8);
		f_3 = - (std::pow(delta_t, 2) / 2);
		f_4 = (std::pow(delta_t, 3) / 6);
	}
	else
	{
		f_1 = (w_dt * cos_wt - sin_wt) / (std::pow(mag_w, 3));
		f_2 = (std::pow(w_dt, 2) - 2 * cos_wt - 2 * w_dt * sin_wt + 2) / (2 * std::pow(mag_w, 4));
		f_3 = - (1 - cos_wt) / std::pow(mag_w, 2);
		f_4 = (w_dt - sin_wt) / std::pow(mag_w, 3);
	}

	Eigen::Matrix<double, 3, 3> alpha_arg = ((dt_2 / 2.0) * Eigen::Matrix<double, 3, 3>::Identity() + f_1 * w_x + f_2 * w_x_2);
	Eigen::Matrix<double, 3, 3> Beta_arg = (delta_t * Eigen::Matrix<double, 3, 3>::Identity() + f_3 * w_x + f_4 * w_x_2);

	Eigen::Matrix<double, 3, 3> H_al = R_k_tau1 * alpha_arg;
	Eigen::Matrix<double, 3, 3> H_be = R_k_tau1 * Beta_arg;

	alpha_tau += beta_tau * delta_t + H_al * a_hat;
	beta_tau += H_be * a_hat;

	Eigen::Matrix<double, 3, 3> J_r_tau1 = small_w ? Eigen::Matrix<double, 3, 3>::Identity() - 0.5 * w_tx + (1.0 / 6.0) * w_tx * w_tx 
		: Eigen::Matrix<double, 3, 3>::Identity() - ((1 - cos_wt) / (std::pow((w_dt), 2.0))) * w_tx + ((w_dt - sin_wt) / (std::pow(w_dt, 3.0))) * w_tx * w_tx;

	Eigen::Matrix<double, 3, 3> J_save = J_q;
	J_q = R_tau1_tau2 * J_q + J_r_tau1 * delta_t;

	H_a -= H_al;
	H_a += delta_t * H_b;
	H_b -= H_be;

	Eigen::Matrix<double, 3, 1> g_k = quatType::quatToRot(q_k_lin) * grav;
	O_a += delta_t * O_b;
	O_a += -H_al * R_tau_k * quatType::skewSymmetric(g_k);
	O_b += -H_be * R_tau_k * quatType::skewSymmetric(g_k);

	Eigen::MatrixXd d_R_bw_1 = - R_k_tau1 * quatType::skewSymmetric(J_q * e_1);
	Eigen::MatrixXd d_R_bw_2 = - R_k_tau1 * quatType::skewSymmetric(J_q * e_2);
	Eigen::MatrixXd d_R_bw_3 = - R_k_tau1 * quatType::skewSymmetric(J_q * e_3);

	double df_1_dbw_1;
	double df_1_dbw_2;
	double df_1_dbw_3;

	double df_2_dbw_1;
	double df_2_dbw_2;
	double df_2_dbw_3;

	double df_3_dbw_1;
	double df_3_dbw_2;
	double df_3_dbw_3;

	double df_4_dbw_1;
	double df_4_dbw_2;
	double df_4_dbw_3;

	if (small_w)
	{
		double df_1_dw_mag = -(std::pow(delta_t, 5) / 15);
		df_1_dbw_1 = w_1 * df_1_dw_mag;
		df_1_dbw_2 = w_2 * df_1_dw_mag;
		df_1_dbw_3 = w_3 * df_1_dw_mag;

		double df_2_dw_mag = (std::pow(delta_t, 6) / 72);
		df_2_dbw_1 = w_1 * df_2_dw_mag;
		df_2_dbw_2 = w_2 * df_2_dw_mag;
		df_2_dbw_3 = w_3 * df_2_dw_mag;

		double df_3_dw_mag = -(std::pow(delta_t, 4) / 12);
		df_3_dbw_1 = w_1 * df_3_dw_mag;
		df_3_dbw_2 = w_2 * df_3_dw_mag;
		df_3_dbw_3 = w_3 * df_3_dw_mag;

		double df_4_dw_mag = (std::pow(delta_t, 5) / 60);
		df_4_dbw_1 = w_1 * df_4_dw_mag;
		df_4_dbw_2 = w_2 * df_4_dw_mag;
		df_4_dbw_3 = w_3 * df_4_dw_mag;
	}
	else
	{
		double df_1_dw_mag = (std::pow(w_dt, 2) * sin_wt - 3 * sin_wt + 3 * w_dt * cos_wt) / std::pow(mag_w, 5);
		df_1_dbw_1 = w_1 * df_1_dw_mag;
		df_1_dbw_2 = w_2 * df_1_dw_mag;
		df_1_dbw_3 = w_3 * df_1_dw_mag;

		double df_2_dw_mag = (std::pow(w_dt, 2) - 4 * cos_wt - 4 * w_dt * sin_wt + std::pow(w_dt, 2) * cos_wt + 4) / (std::pow(mag_w, 6));
		df_2_dbw_1 = w_1 * df_2_dw_mag;
		df_2_dbw_2 = w_2 * df_2_dw_mag;
		df_2_dbw_3 = w_3 * df_2_dw_mag;

		double df_3_dw_mag = (2 * (cos_wt - 1) + w_dt * sin_wt) / (std::pow(mag_w, 4));
		df_3_dbw_1 = w_1 * df_3_dw_mag;
		df_3_dbw_2 = w_2 * df_3_dw_mag;
		df_3_dbw_3 = w_3 * df_3_dw_mag;

		double df_4_dw_mag = (2 * w_dt + w_dt * cos_wt - 3 * sin_wt) / (std::pow(mag_w, 5));
		df_4_dbw_1 = w_1 * df_4_dw_mag;
		df_4_dbw_2 = w_2 * df_4_dw_mag;
		df_4_dbw_3 = w_3 * df_4_dw_mag;
	}

	Eigen::Matrix<double, 3, 1> g_tau = R_tau_k * quatType::quatToRot(q_k_lin) * grav;

	J_a += J_b * delta_t;
	J_a.block(0, 0, 3, 1) +=
		(d_R_bw_1 * alpha_arg + R_k_tau1 * (df_1_dbw_1 * w_x - f_1 * e_1x + df_2_dbw_1 * w_x_2 - f_2 * (e_1x * w_x + w_x * e_1x))) * a_hat -
		H_al * quatType::skewSymmetric((J_save * e_1)) * g_tau;
	J_a.block(0, 1, 3, 1) +=
		(d_R_bw_2 * alpha_arg + R_k_tau1 * (df_1_dbw_2 * w_x - f_1 * e_2x + df_2_dbw_2 * w_x_2 - f_2 * (e_2x * w_x + w_x * e_2x))) * a_hat -
		H_al * quatType::skewSymmetric((J_save * e_2)) * g_tau;
	J_a.block(0, 2, 3, 1) +=
		(d_R_bw_3 * alpha_arg + R_k_tau1 * (df_1_dbw_3 * w_x - f_1 * e_3x + df_2_dbw_3 * w_x_2 - f_2 * (e_3x * w_x + w_x * e_3x))) * a_hat -
		H_al * quatType::skewSymmetric((J_save * e_3)) * g_tau;
	J_b.block(0, 0, 3, 1) +=
		(d_R_bw_1 * Beta_arg + R_k_tau1 * (df_3_dbw_1 * w_x - f_3 * e_1x + df_4_dbw_1 * w_x_2 - f_4 * (e_1x * w_x + w_x * e_1x))) * a_hat -
		-H_be * quatType::skewSymmetric((J_save * e_1)) * g_tau;
	J_b.block(0, 1, 3, 1) +=
		(d_R_bw_2 * Beta_arg + R_k_tau1 * (df_3_dbw_2 * w_x - f_3 * e_2x + df_4_dbw_2 * w_x_2 - f_4 * (e_2x * w_x + w_x * e_2x))) * a_hat -
		H_be * quatType::skewSymmetric((J_save * e_2)) * g_tau;
	J_b.block(0, 2, 3, 1) +=
		(d_R_bw_3 * Beta_arg + R_k_tau1 * (df_3_dbw_3 * w_x - f_3 * e_3x + df_4_dbw_3 * w_x_2 - f_4 * (e_3x * w_x + w_x * e_3x))) * a_hat -
		H_be * quatType::skewSymmetric((J_save * e_3)) * g_tau;

	Eigen::Matrix<double, 3, 3> R_G_to_k = quatType::quatToRot(q_k_lin);
	double dt_mid = delta_t / 2.0;
	double w_dt_mid = mag_w * dt_mid;
	Eigen::Matrix<double, 3, 3> R_mid;

	R_mid = small_w ? Eigen::Matrix<double, 3, 3>::Identity() - dt_mid * w_x + (std::pow(dt_mid, 2) / 2) * w_x_2
		: Eigen::Matrix<double, 3, 3>::Identity() - (std::sin(w_dt_mid) / mag_w) * w_x + ((1.0 - std::cos(w_dt_mid)) / (std::pow(mag_w, 2.0))) * w_x_2;
	R_mid = R_mid * R_tau_k;

	Eigen::Matrix<double, 21, 21> F_k1 = Eigen::Matrix<double, 21, 21>::Zero();
	F_k1.block(0, 0, 3, 3) = - w_x;
	F_k1.block(0, 3, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	F_k1.block(6, 0, 3, 3) = - R_tau_k.transpose() * a_x;
	F_k1.block(6, 9, 3, 3) = - R_tau_k.transpose();
	F_k1.block(6, 15, 3, 3) = - R_tau_k.transpose() * quatType::skewSymmetric(R_tau_k * R_G_to_k * grav);
	F_k1.block(6, 18, 3, 3) = - R_tau_k.transpose() * R_tau_k * quatType::skewSymmetric(R_G_to_k * grav);
	F_k1.block(12, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 21, 12> G_k1 = Eigen::Matrix<double, 21, 12>::Zero();
	G_k1.block(0, 0, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	G_k1.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	G_k1.block(6, 6, 3, 3) = - R_tau_k.transpose();
	G_k1.block(9, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 21, 21> Phi_dot_k1 = F_k1;
	Eigen::Matrix<double, 21, 21> P_dot_k1 = F_k1 * P_big + P_big * F_k1.transpose() + G_k1 * Q_c * G_k1.transpose();

	Eigen::Matrix<double, 21, 21> F_k2 = Eigen::Matrix<double, 21, 21>::Zero();
	F_k2.block(0, 0, 3, 3) = - w_x;
	F_k2.block(0, 3, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	F_k2.block(6, 0, 3, 3) = - R_mid.transpose() * a_x;
	F_k2.block(6, 9, 3, 3) = - R_mid.transpose();
	F_k2.block(6, 15, 3, 3) = - R_mid.transpose() * quatType::skewSymmetric(R_tau_k * R_G_to_k * grav);
	F_k2.block(6, 18, 3, 3) = - R_mid.transpose() * R_tau_k * quatType::skewSymmetric(R_G_to_k * grav);
	F_k2.block(12, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 21, 12> G_k2 = Eigen::Matrix<double, 21, 12>::Zero();
	G_k2.block(0, 0, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	G_k2.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	G_k2.block(6, 6, 3, 3) = - R_mid.transpose();
	G_k2.block(9, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 21, 21> Phi_k2 = Eigen::Matrix<double, 21, 21>::Identity() + Phi_dot_k1 * dt_mid;
	Eigen::Matrix<double, 21, 21> P_k2 = P_big + P_dot_k1 * dt_mid;
	Eigen::Matrix<double, 21, 21> Phi_dot_k2 = F_k2 * Phi_k2;
	Eigen::Matrix<double, 21, 21> P_dot_k2 = F_k2 * P_k2 + P_k2 * F_k2.transpose() + G_k2 * Q_c * G_k2.transpose();

	Eigen::Matrix<double, 21, 21> F_k3 = F_k2;
	Eigen::Matrix<double, 21, 12> G_k3 = G_k2;

	Eigen::Matrix<double, 21, 21> Phi_k3 = Eigen::Matrix<double, 21, 21>::Identity() + Phi_dot_k2 * dt_mid;
	Eigen::Matrix<double, 21, 21> P_k3 = P_big + P_dot_k2 * dt_mid;
	Eigen::Matrix<double, 21, 21> Phi_dot_k3 = F_k3 * Phi_k3;
	Eigen::Matrix<double, 21, 21> P_dot_k3 = F_k3 * P_k3 + P_k3 * F_k3.transpose() + G_k3 * Q_c * G_k3.transpose();

	Eigen::Matrix<double, 21, 21> F_k4 = Eigen::Matrix<double, 21, 21>::Zero();
	F_k4.block(0, 0, 3, 3) = - w_x;
	F_k4.block(0, 3, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	F_k4.block(6, 0, 3, 3) = - R_tau1_k.transpose() * a_x;
	F_k4.block(6, 9, 3, 3) = - R_tau1_k.transpose();
	F_k4.block(6, 15, 3, 3) = - R_tau1_k.transpose() * quatType::skewSymmetric(R_tau_k * R_G_to_k * grav);
	F_k4.block(6, 18, 3, 3) = - R_tau1_k.transpose() * R_tau_k * quatType::skewSymmetric(R_G_to_k * grav);
	F_k4.block(12, 6, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 21, 12> G_k4 = Eigen::Matrix<double, 21, 12>::Zero();
	G_k4.block(0, 0, 3, 3) = - Eigen::Matrix<double, 3, 3>::Identity();
	G_k4.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	G_k4.block(6, 6, 3, 3) = - R_tau1_k.transpose();
	G_k4.block(9, 9, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	Eigen::Matrix<double, 21, 21> Phi_k4 = Eigen::Matrix<double, 21, 21>::Identity() + Phi_dot_k3 * delta_t;
	Eigen::Matrix<double, 21, 21> P_k4 = P_big + P_dot_k3 * delta_t;
	Eigen::Matrix<double, 21, 21> Phi_dot_k4 = F_k4 * Phi_k4;
	Eigen::Matrix<double, 21, 21> P_dot_k4 = F_k4 * P_k4 + P_k4 * F_k4.transpose() + G_k4 * Q_c * G_k4.transpose();

	P_big += (delta_t / 6.0) * (P_dot_k1 + 2.0 * P_dot_k2 + 2.0 * P_dot_k3 + P_dot_k4);
	P_big = 0.5 * (P_big + P_big.transpose());

	Eigen::Matrix<double, 21, 21> Phi =
		Eigen::Matrix<double, 21, 21>::Identity() + (delta_t / 6.0) * (Phi_dot_k1 + 2.0 * Phi_dot_k2 + 2.0 * Phi_dot_k3 + Phi_dot_k4);

	Eigen::Matrix<double, 21, 21> B_k = Eigen::Matrix<double, 21, 21>::Identity();
	B_k.block(15, 15, 3, 3).setZero();
	B_k.block(15, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

	P_big = B_k * P_big * B_k.transpose();
	P_big = 0.5 * (P_big + P_big.transpose());
	Discrete_J_b = B_k * Phi * Discrete_J_b;

	P_meas = P_big.block(0, 0, 15, 15);

	if (state_transition_jacobians)
	{
		J_q = - Discrete_J_b.block(0, 3, 3, 3);
		J_a = Discrete_J_b.block(12, 3, 3, 3);
		J_b = Discrete_J_b.block(6, 3, 3, 3);
		H_a = Discrete_J_b.block(12, 9, 3, 3);
		H_b = Discrete_J_b.block(6, 9, 3, 3);
		O_a = Discrete_J_b.block(12, 18, 3, 3);
		O_b = Discrete_J_b.block(6, 18, 3, 3);
	}

	R_tau_k = R_tau1_k;
	q_tau_k = quatType::rotToQuat(R_tau_k);
}



bool stateJplQuatLocal::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
	Eigen::Map<const Eigen::Vector4d> q(x);

	Eigen::Map<const Eigen::Vector3d> d_th(delta);
	Eigen::Matrix<double, 4, 1> d_q;
	double theta = d_th.norm();

	if (theta < 1e-8)
		d_q << 0.5 * d_th, 1.0;
	else
	{
		d_q.block(0, 0, 3, 1) = (d_th / theta) * std::sin(theta / 2);
		d_q(3, 0) = std::cos(theta / 2);
	}
	d_q = quatType::quatNorm(d_q);

	Eigen::Map<Eigen::Vector4d> q_plus(x_plus_delta);
	q_plus = quatType::quatMultiply(d_q, q);

	return true;
}

bool stateJplQuatLocal::ComputeJacobian(const double *x, double *jacobian) const
{
	Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
	j.topRows<3>().setIdentity();
	j.bottomRows<1>().setZero();

	return true;
}



factorGenericPrior::factorGenericPrior(const Eigen::MatrixXd &x_lin_, const std::vector<std::string> &x_type_, 
	const Eigen::MatrixXd &prior_Info, const Eigen::MatrixXd &prior_grad) : x_lin(x_lin_), x_type(x_type_)
{
	int state_size = 0;
	int state_error_size = 0;

	for (auto const &str : x_type_)
	{
		if (str == "quat")
		{
			state_size += 4;
			state_error_size += 3;
		}
		else if (str == "quat_yaw")
		{
			state_size += 4;
			state_error_size += 1;
		}
		else if (str == "vec1")
		{
			state_size += 1;
			state_error_size += 1;
		}
		else if (str == "vec3")
		{
			state_size += 3;
			state_error_size += 3;
		}
		else if (str == "vec8")
		{
			state_size += 8;
			state_error_size += 8;
		}
		else
		{
			std::cerr << "type - " << str << " not implemented in prior" << std::endl;
			std::exit(EXIT_FAILURE);
		}
	}

	assert(x_lin.rows() == state_size);
	assert(x_lin.cols() == 1);
	assert(prior_Info.rows() == state_error_size);
	assert(prior_Info.cols() == state_error_size);
	assert(prior_grad.rows() == state_error_size);
	assert(prior_grad.cols() == 1);

	Eigen::LLT<Eigen::MatrixXd> llt_of_I(prior_Info);
	sqrt_I = llt_of_I.matrixL().transpose();
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(prior_Info.rows(), prior_Info.rows());
	b = sqrt_I.triangularView<Eigen::Upper>().solve(I) * prior_grad;

	if (std::isnan(prior_Info.norm()) || std::isnan(sqrt_I.norm()) || std::isnan(b.norm()))
	{
		std::cerr << "prior_Info - " << std::endl << prior_Info << std::endl << std::endl;
		std::cerr << "prior_Info_inv - " << std::endl << prior_Info.inverse() << std::endl << std::endl;
		std::cerr << "b - " << std::endl << b << std::endl << std::endl;
		std::exit(EXIT_FAILURE);
	}

	set_num_residuals(state_error_size);

	for (auto const &str : x_type_)
	{
		if (str == "quat")
			mutable_parameter_block_sizes()->push_back(4);
		if (str == "quat_yaw")
			mutable_parameter_block_sizes()->push_back(4);
		if (str == "vec1")
			mutable_parameter_block_sizes()->push_back(1);
		if (str == "vec3")
			mutable_parameter_block_sizes()->push_back(3);
		if (str == "vec8")
			mutable_parameter_block_sizes()->push_back(8);
	}
}

bool factorGenericPrior::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
	int local_it = 0;
	int global_it = 0;
	Eigen::MatrixXd res = Eigen::MatrixXd::Zero(num_residuals(), 1);

	for (size_t i = 0; i < x_type.size(); i++)
	{
		if (x_type[i] == "quat")
		{
			Eigen::Vector4d q_i = Eigen::Map<const Eigen::Vector4d>(parameters[i]);
			Eigen::Matrix3d R_i = quatType::quatToRot(q_i);
			Eigen::Matrix3d R_lin = quatType::quatToRot(x_lin.block(global_it, 0, 4, 1));
			Eigen::Vector3d theta_err = quatType::logSo3(R_i.transpose() * R_lin);
			res.block(local_it, 0, 3, 1) = - theta_err;

			if (jacobians && jacobians[i])
			{
				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 4);
				jacobian.setZero();
				Eigen::Matrix3d Jr_inv = quatType::JrighySo3(theta_err).inverse();
				Eigen::Matrix3d H_theta = - Jr_inv * R_lin.transpose();
				jacobian.block(0, 0, num_residuals(), 3) = sqrt_I.block(0, local_it, num_residuals(), 3) * H_theta;
			}

			global_it += 4;
			local_it += 3;
		}
		else if (x_type[i] == "quat_yaw")
		{
			Eigen::Vector3d ez = Eigen::Vector3d(0.0, 0.0, 1.0);
			Eigen::Vector4d q_i = Eigen::Map<const Eigen::Vector4d>(parameters[i]);
			Eigen::Matrix3d R_i = quatType::quatToRot(q_i);
			Eigen::Matrix3d R_lin = quatType::quatToRot(x_lin.block(global_it, 0, 4, 1));
			Eigen::Vector3d theta_err = quatType::logSo3(R_i.transpose() * R_lin);
			res(local_it, 0) = - (ez.transpose() * theta_err)(0, 0);

			if (jacobians && jacobians[i])
			{
				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 4);
				jacobian.setZero();
				Eigen::Matrix3d Jr_inv = quatType::JrighySo3(theta_err).inverse();
				Eigen::Matrix<double, 1, 3> H_theta = - ez.transpose() * (Jr_inv * R_lin.transpose());
				jacobian.block(0, 0, num_residuals(), 3) = sqrt_I.block(0, local_it, num_residuals(), 1) * H_theta;
			}

			global_it += 4;
			local_it += 1;
		}
		else if (x_type[i] == "vec1")
		{
			double x = parameters[i][0];
			res(local_it, 0) = x - x_lin(global_it, 0);

			if (jacobians && jacobians[i])
			{
				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J_vi(jacobians[i], num_residuals(), 1);
				J_vi.block(0, 0, num_residuals(), 1) = sqrt_I.block(0, local_it, num_residuals(), 1);
			}

			global_it += 1;
			local_it += 1;
		}
		else if (x_type[i] == "vec3")
		{
			Eigen::Matrix<double, 3, 1> p_i = Eigen::Map<const Eigen::Matrix<double, 3, 1>>(parameters[i]);
			res.block(local_it, 0, 3, 1) = p_i - x_lin.block(global_it, 0, 3, 1);

			if (jacobians && jacobians[i])
			{
				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 3);
				jacobian.block(0, 0, num_residuals(), 3) = sqrt_I.block(0, local_it, num_residuals(), 3);
			}

			global_it += 3;
			local_it += 3;
		}
		else if (x_type[i] == "vec8")
		{
			Eigen::Matrix<double, 8, 1> p_i = Eigen::Map<const Eigen::Matrix<double, 8, 1>>(parameters[i]);
			res.block(local_it, 0, 8, 1) = p_i - x_lin.block(global_it, 0, 8, 1);

			if (jacobians && jacobians[i])
			{
				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 8);
				jacobian.block(0, 0, num_residuals(), 8) = sqrt_I.block(0, local_it, num_residuals(), 8);
			}

			global_it += 8;
			local_it += 8;
		}
		else
		{
			std::cerr << "type - " << x_type[i] << " not implemented in prior" << std::endl;
			std::exit(EXIT_FAILURE);
		}
	}

	res = sqrt_I * res;
	res += b;

	for (int i = 0; i < res.rows(); i++)
		residuals[i] = res(i, 0);

	return true;
}



factorImuPreIntegrationV1::factorImuPreIntegrationV1(double deltatime, Eigen::Vector3d &grav, Eigen::Vector3d &alpha, Eigen::Vector3d &beta,
	Eigen::Vector4d &q_KtoK1, Eigen::Vector3d &ba_lin, Eigen::Vector3d &bg_lin, Eigen::Matrix3d &J_q, 
	Eigen::Matrix3d &J_beta, Eigen::Matrix3d &J_alpha, Eigen::Matrix3d &H_beta, Eigen::Matrix3d &H_alpha, 
	Eigen::Matrix<double, 15, 15> &covariance)
{
	this->alpha = alpha;
	this->beta = beta;
	this->q_breve = q_KtoK1;
	this->dt = deltatime;
	this->grav_save = grav;

	this->b_a_lin_save = ba_lin;
	this->b_w_lin_save = bg_lin;

	this->J_q = J_q;
	this->J_a = J_alpha;
	this->J_b = J_beta;
	this->H_a = H_alpha;
	this->H_b = H_beta;

	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(covariance.rows(), covariance.rows());
	Eigen::MatrixXd information = covariance.llt().solve(I);
  
	if (std::isnan(information.norm()))
	{
		std::cerr << "P - " << std::endl << covariance << std::endl << std::endl;
		std::cerr << "Pinv - " << std::endl << covariance.inverse() << std::endl << std::endl;
		std::exit(EXIT_FAILURE);
	}

	Eigen::LLT<Eigen::MatrixXd> llt_of_I(information);
	sqrt_I_save = llt_of_I.matrixL().transpose();

	set_num_residuals(15);
	mutable_parameter_block_sizes()->push_back(4);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(4);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(3);
}

bool factorImuPreIntegrationV1::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
	Eigen::Vector3d gravity = grav_save;
	Eigen::Matrix<double, 15, 15> sqrt_I = sqrt_I_save;
	Eigen::Vector3d b_w_lin = b_w_lin_save;
	Eigen::Vector3d b_a_lin = b_a_lin_save;

	Eigen::Vector4d q_1 = Eigen::Map<const Eigen::Vector4d>(parameters[0]);
	Eigen::Matrix3d R_1 = quatType::quatToRot(q_1);
	Eigen::Vector4d q_2 = Eigen::Map<const Eigen::Vector4d>(parameters[5]);

	Eigen::Vector3d b_w1 = Eigen::Map<const Eigen::Vector3d>(parameters[1]);
	Eigen::Vector3d b_w2 = Eigen::Map<const Eigen::Vector3d>(parameters[6]);

	Eigen::Vector3d b_a1 = Eigen::Map<const Eigen::Vector3d>(parameters[3]);
	Eigen::Vector3d b_a2 = Eigen::Map<const Eigen::Vector3d>(parameters[8]);

	Eigen::Vector3d v_1 = Eigen::Map<const Eigen::Vector3d>(parameters[2]);
	Eigen::Vector3d v_2 = Eigen::Map<const Eigen::Vector3d>(parameters[7]);

	Eigen::Vector3d p_1 = Eigen::Map<const Eigen::Vector3d>(parameters[4]);
	Eigen::Vector3d p_2 = Eigen::Map<const Eigen::Vector3d>(parameters[9]);

	Eigen::Vector3d dbw = b_w1 - b_w_lin;
	Eigen::Vector3d dba = b_a1 - b_a_lin;

	Eigen::Vector4d q_b;
	q_b.block(0, 0, 3, 1) = 0.5 * J_q * dbw;
	q_b(3, 0) = 1.0;
	q_b = q_b / q_b.norm();

	Eigen::Vector4d q_1_to_2 = quatType::quatMultiply(q_2, quatType::inv(q_1));

	Eigen::Vector4d q_res_minus = quatType::quatMultiply(q_1_to_2, quatType::inv(q_breve));
	Eigen::Vector4d q_res_plus = quatType::quatMultiply(q_res_minus, q_b);

	Eigen::Matrix<double, 15, 1> res;
	res.block(0, 0, 3, 1) = 2 * q_res_plus.block(0, 0, 3, 1);
	res.block(3, 0, 3, 1) = b_w2 - b_w1;
	res.block(6, 0, 3, 1) = R_1 * (v_2 - v_1 + gravity * dt) - J_b * dbw - H_b * dba - beta;
	res.block(9, 0, 3, 1) = b_a2 - b_a1;
	res.block(12, 0, 3, 1) = R_1 * (p_2 - p_1 - v_1 * dt + .5 * gravity * std::pow(dt, 2)) - J_a * dbw - H_a * dba - alpha;
	res = sqrt_I * res;

	for (int i = 0; i < res.rows(); i++)
		residuals[i] = res(i, 0);

	if (jacobians)
	{
		Eigen::Matrix<double, 15, 30> Jacobian = Eigen::Matrix<double, 15, 30>::Zero();

		Eigen::Matrix<double, 4, 1> q_meas_plus = quatType::quatMultiply(quatType::inv(q_breve), q_b);

		Jacobian.block(0, 0, 3, 3) = - ((q_1_to_2(3, 0) * Eigen::MatrixXd::Identity(3, 3) - quatType::skewSymmetric(q_1_to_2.block(0, 0, 3, 1))) * 
			(q_meas_plus(3, 0) * Eigen::MatrixXd::Identity(3, 3) + quatType::skewSymmetric(q_meas_plus.block(0, 0, 3, 1))) -
			q_1_to_2.block(0, 0, 3, 1) * q_meas_plus.block(0, 0, 3, 1).transpose());

		Jacobian.block(0, 15, 3, 3) = q_res_plus(3, 0) * Eigen::MatrixXd::Identity(3, 3) + quatType::skewSymmetric(q_res_plus.block(0, 0, 3, 1));

		Jacobian.block(0, 3, 3, 3) = (q_res_minus(3, 0) * Eigen::MatrixXd::Identity(3, 3) - quatType::skewSymmetric(q_res_minus.block(0, 0, 3, 1))) * J_q;

		Jacobian.block(3, 3, 3, 3) = - Eigen::MatrixXd::Identity(3, 3);
		Jacobian.block(3, 18, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

		Jacobian.block(6, 0, 3, 3) = quatType::skewSymmetric(R_1 * (v_2 - v_1 + gravity * dt));

		Jacobian.block(6, 6, 3, 3) = - R_1;

		Jacobian.block(6, 21, 3, 3) = R_1;

		Jacobian.block(6, 3, 3, 3) = - J_b;

		Jacobian.block(6, 9, 3, 3) = - H_b;

		Jacobian.block(9, 9, 3, 3) = - Eigen::MatrixXd::Identity(3, 3);
		Jacobian.block(9, 24, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

		Jacobian.block(12, 0, 3, 3) = quatType::skewSymmetric(R_1 * (p_2 - p_1 - v_1 * dt + 0.5 * gravity * std::pow(dt, 2)));
		Jacobian.block(12, 6, 3, 3) = - R_1 * dt;
		Jacobian.block(12, 12, 3, 3) = - R_1;

		Jacobian.block(12, 27, 3, 3) = R_1;

		Jacobian.block(12, 3, 3, 3) = - J_a;
		Jacobian.block(12, 9, 3, 3) = - H_a;

		Jacobian = sqrt_I * Jacobian;

		if (jacobians[0])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> J_th1(jacobians[0], 15, 4);
			J_th1.block(0, 0, 15, 3) = Jacobian.block(0, 0, 15, 3);
			J_th1.block(0, 3, 15, 1).setZero();
		}

		if (jacobians[5])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> J_th2(jacobians[5], 15, 4);
			J_th2.block(0, 0, 15, 3) = Jacobian.block(0, 15, 15, 3);
			J_th2.block(0, 3, 15, 1).setZero();
		}

		if (jacobians[1])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_bw1(jacobians[1], 15, 3);
			J_bw1.block(0, 0, 15, 3) = Jacobian.block(0, 3, 15, 3);
		}

		if (jacobians[6])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_bw2(jacobians[6], 15, 3);
			J_bw2.block(0, 0, 15, 3) = Jacobian.block(0, 18, 15, 3);
		}

		if (jacobians[2])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_v1(jacobians[2], 15, 3);
			J_v1.block(0, 0, 15, 3) = Jacobian.block(0, 6, 15, 3);
		}

		if (jacobians[7])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_v2(jacobians[7], 15, 3);
			J_v2.block(0, 0, 15, 3) = Jacobian.block(0, 21, 15, 3);
		}

		if (jacobians[3])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_ba1(jacobians[3], 15, 3);
			J_ba1.block(0, 0, 15, 3) = Jacobian.block(0, 9, 15, 3);
		}

		if (jacobians[8])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_ba2(jacobians[8], 15, 3);
			J_ba2.block(0, 0, 15, 3) = Jacobian.block(0, 24, 15, 3);
		}

		if (jacobians[4])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_p1(jacobians[4], 15, 3);
			J_p1.block(0, 0, 15, 3) = Jacobian.block(0, 12, 15, 3);
		}

		if (jacobians[9])
		{
			Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_p2(jacobians[9], 15, 3);
			J_p2.block(0, 0, 15, 3) = Jacobian.block(0, 27, 15, 3);
		}
	}

	return true;
}



factorImageReprojCalib::factorImageReprojCalib(const Eigen::Vector2d &uv_meas_, double pix_sigma_, bool is_fisheye_) 
	: uv_meas(uv_meas_), pix_sigma(pix_sigma_), is_fisheye(is_fisheye_)
{
	sqrt_Q = Eigen::Matrix<double, 2, 2>::Identity();
	sqrt_Q(0, 0) *= 1.0 / pix_sigma;
	sqrt_Q(1, 1) *= 1.0 / pix_sigma;

	set_num_residuals(2);
	mutable_parameter_block_sizes()->push_back(4);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(4);
	mutable_parameter_block_sizes()->push_back(3);
	mutable_parameter_block_sizes()->push_back(8);
}

bool factorImageReprojCalib::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
	Eigen::Vector4d q_GtoIi = Eigen::Map<const Eigen::Vector4d>(parameters[0]);
	Eigen::Matrix3d R_GtoIi = quatType::quatToRot(q_GtoIi);
	Eigen::Vector3d p_IiinG = Eigen::Map<const Eigen::Vector3d>(parameters[1]);
	Eigen::Vector3d p_FinG = Eigen::Map<const Eigen::Vector3d>(parameters[2]);
	Eigen::Vector4d q_ItoC = Eigen::Map<const Eigen::Vector4d>(parameters[3]);
	Eigen::Matrix3d R_ItoC = quatType::quatToRot(q_ItoC);
	Eigen::Vector3d p_IinC = Eigen::Map<const Eigen::Vector3d>(parameters[4]);

	Eigen::Matrix<double, 8, 1> camera_vals = Eigen::Map<const Eigen::Matrix<double, 8, 1>>(parameters[5]);

	Eigen::Vector3d p_FinIi = R_GtoIi * (p_FinG - p_IiinG);
	Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;

	Eigen::Vector2d uv_norm;
	uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);

	Eigen::Matrix<double, 2, 2> sqrt_Q_gate = gate * sqrt_Q;

	Eigen::Vector2d uv_dist;
	Eigen::MatrixXd H_dz_dzn, H_dz_dzeta;

	if (is_fisheye)
	{
		cameraEqui cam(0, 0);
		cam.setValue(camera_vals);
		uv_dist = cam.distortD(uv_norm);
		if (jacobians)
		{
			cam.computeDistortJacobian(uv_norm, H_dz_dzn, H_dz_dzeta);
			H_dz_dzn = sqrt_Q_gate * H_dz_dzn;
			H_dz_dzeta = sqrt_Q_gate * H_dz_dzeta;
		}
	}
	else
	{
		cameraRadtan cam(0, 0);
		cam.setValue(camera_vals);
		uv_dist = cam.distortD(uv_norm);
		if (jacobians)
		{
			cam.computeDistortJacobian(uv_norm, H_dz_dzn, H_dz_dzeta);
			H_dz_dzn = sqrt_Q_gate * H_dz_dzn;
			H_dz_dzeta = sqrt_Q_gate * H_dz_dzeta;
		}
	}

	Eigen::Vector2d res = uv_dist - uv_meas;
	res = sqrt_Q_gate * res;
	residuals[0] = res(0);
	residuals[1] = res(1);

	if (jacobians)
	{
		Eigen::MatrixXd H_dzn_dpfc = Eigen::MatrixXd::Zero(2, 3);
		H_dzn_dpfc << 1.0 / p_FinCi(2), 0, -p_FinCi(0) / std::pow(p_FinCi(2), 2), 0, 1.0 / p_FinCi(2), -p_FinCi(1) / std::pow(p_FinCi(2), 2);
		Eigen::MatrixXd H_dz_dpfc = H_dz_dzn * H_dzn_dpfc;

		if (jacobians[0])
		{
			Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian(jacobians[0]);
			jacobian.block(0, 0, 2, 3) = H_dz_dpfc * R_ItoC * quatType::skewSymmetric(p_FinIi);
			jacobian.block(0, 3, 2, 1).setZero();
		}

		if (jacobians[1]) {
			Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[1]);
			jacobian.block(0, 0, 2, 3) = -H_dz_dpfc * R_ItoC * R_GtoIi;
		}

		if (jacobians[2])
		{
			Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[2]);
			jacobian.block(0, 0, 2, 3) = H_dz_dpfc * R_ItoC * R_GtoIi;
		}

		if (jacobians[3])
		{
			Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian(jacobians[3]);
			jacobian.block(0, 0, 2, 3) = H_dz_dpfc * quatType::skewSymmetric(R_ItoC * p_FinIi);
			jacobian.block(0, 3, 2, 1).setZero();
		}

		if (jacobians[4])
		{
			Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[4]);
			jacobian.block(0, 0, 2, 3) = H_dz_dpfc;
		}

		if (jacobians[5])
		{
			Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>> jacobian(jacobians[5]);
			jacobian.block(0, 0, 2, 8) = H_dz_dzeta;
		}
	}
	
	return true;
}