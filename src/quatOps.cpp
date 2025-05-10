#include "quatOps.h"

Eigen::Matrix<double, 4, 1> quatType::rotToQuat(const Eigen::Matrix<double, 3, 3> &rot)
{
	Eigen::Matrix<double, 4, 1> q;
	double T = rot.trace();

	if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) && (rot(0, 0) >= rot(2, 2)))
	{
		q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
		q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
		q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
		q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));
	} 
	else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2)))
	{
		q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
		q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
		q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
		q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
	}
	else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) && (rot(2, 2) >= rot(1, 1)))
	{
		q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
		q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
		q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
		q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
	} 
	else
	{
		q(3) = sqrt((1 + T) / 4);
		q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
		q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
		q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
	}

	if (q(3) < 0)
	{
		q = -q;
	}

	q = q / (q.norm());
	return q;
}

Eigen::Matrix<double, 3, 3> quatType::quatToRot(const Eigen::Matrix<double, 4, 1> &q)
{
	Eigen::Matrix<double, 3, 3> q_x = skewSymmetric(q.block<3, 1>(0, 0));
	Eigen::MatrixXd rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                    	   2 * q.block<3, 1>(0, 0) * (q.block<3, 1>(0, 0).transpose());
	return rot;
}

Eigen::Matrix<double, 3, 3> quatType::skewSymmetric(const Eigen::Matrix<double, 3, 1> &w)
{
	Eigen::Matrix<double, 3, 3> w_x;
	w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
	return w_x;
}

Eigen::Matrix<double, 4, 1> quatType::quatMultiply(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p)
{
	Eigen::Matrix<double, 4, 1> q_t;
	Eigen::Matrix<double, 4, 4> Q_m;

	Q_m.block<3, 3>(0, 0) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skewSymmetric(q.block<3, 1>(0, 0));
	Q_m.block<3, 1>(0, 3) = q.block<3, 1>(0, 0);
	Q_m.block<1, 3>(3, 0) = -q.block<3, 1>(0, 0).transpose();
	Q_m(3, 3) = q(3, 0);
	q_t = Q_m * p;

	if (q_t(3, 0) < 0)
	{
		q_t *= -1;
	}

	return q_t / q_t.norm();
}

Eigen::Matrix<double, 3, 1> quatType::vee(const Eigen::Matrix<double, 3, 3> &w_x)
{
	Eigen::Matrix<double, 3, 1> w;
	w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
	return w;
}

Eigen::Matrix<double, 3, 3> quatType::expSo3(const Eigen::Matrix<double, 3, 1> &w)
{
	Eigen::Matrix<double, 3, 3> w_x = skewSymmetric(w);
	double theta = w.norm();

	double A, B;
	if (theta < 1e-7) {
		A = 1;
		B = 0.5;
	}
	else
	{
		A = sin(theta) / theta;
		B = (1 - cos(theta)) / (theta * theta);
	}

	Eigen::Matrix<double, 3, 3> R;
	if (theta == 0) {
		R = Eigen::MatrixXd::Identity(3, 3);
	} else {
		R = Eigen::MatrixXd::Identity(3, 3) + A * w_x + B * w_x * w_x;
	}
	return R;
}

Eigen::Matrix<double, 3, 1> quatType::logSo3(const Eigen::Matrix<double, 3, 3> &R)
{
	double R_11 = R(0, 0), R_12 = R(0, 1), R_13 = R(0, 2);
	double R_21 = R(1, 0), R_22 = R(1, 1), R_23 = R(1, 2);
	double R_31 = R(2, 0), R_32 = R(2, 1), R_33 = R(2, 2);

	const double tr = R.trace();
	Eigen::Vector3d omega;

	if (tr + 1.0 < 1e-10)
	{
    	if (std::abs(R_33 + 1.0) > 1e-5)
			omega = (M_PI / sqrt(2.0 + 2.0 * R_33)) * Eigen::Vector3d(R_13, R_23, 1.0 + R_33);
		else if (std::abs(R_22 + 1.0) > 1e-5)
			omega = (M_PI / sqrt(2.0 + 2.0 * R_22)) * Eigen::Vector3d(R_12, 1.0 + R_22, R_32);
		else
			omega = (M_PI / sqrt(2.0 + 2.0 * R_11)) * Eigen::Vector3d(1.0 + R_11, R_21, R_31);
	}
	else
	{
		double magnitude;
		const double tr_3 = tr - 3.0;
		if (tr_3 < -1e-7)
		{
			double theta = acos((tr - 1.0) / 2.0);
			magnitude = theta / (2.0 * sin(theta));
		}
		else
		{
			magnitude = 0.5 - tr_3 / 12.0;
		}
		omega = magnitude * Eigen::Vector3d(R_32 - R_23, R_13 - R_31, R_21 - R_12);
	}
	return omega;
}

Eigen::Matrix4d quatType::expSe3(Eigen::Matrix<double, 6, 1> vec)
{
	Eigen::Vector3d w = vec.head(3);
	Eigen::Vector3d u = vec.tail(3);
	double theta = sqrt(w.dot(w));
	Eigen::Matrix3d wskew;
	wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

	double A, B, C;
	if (theta < 1e-7)
	{
		A = 1;
		B = 0.5;
		C = 1.0 / 6.0;
	}
	else
	{
		A = sin(theta) / theta;
		B = (1 - cos(theta)) / (theta * theta);
		C = (1 - A) / (theta * theta);
	}

	Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d V = I_33 + B * wskew + C * wskew * wskew;

	Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
	mat.block<3, 3>(0, 0) = I_33 + A * wskew + B * wskew * wskew;
	mat.block<3, 1>(0, 3) = V * u;
	mat(3, 3) = 1;
	return mat;
}

Eigen::Matrix<double, 6, 1> quatType::logSe3(Eigen::Matrix4d mat)
{
	Eigen::Vector3d w = logSo3(mat.block<3, 3>(0, 0));
	Eigen::Vector3d T = mat.block<3, 1>(0, 3);
	const double t = w.norm();
	if (t < 1e-10)
	{
		Eigen::Matrix<double, 6, 1> log;
		log << w, T;
		return log;
	}
	else
	{
		Eigen::Matrix3d W = skewSymmetric(w / t);
    	double Tan = tan(0.5 * t);
    	Eigen::Vector3d WT = W * T;
    	Eigen::Vector3d u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
    	Eigen::Matrix<double, 6, 1> log;
    	log << w, u;
    	return log;
	}
}

Eigen::Matrix4d quatType::hatSe3(const Eigen::Matrix<double, 6, 1> &vec)
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
	mat.block<3, 3>(0, 0) = skewSymmetric(vec.head(3));
	mat.block<3, 1>(0, 3) = vec.tail(3);
	return mat;
}

Eigen::Matrix4d quatType::invSe3(const Eigen::Matrix4d &T)
{
	Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
	Tinv.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
	Tinv.block<3, 1>(0, 3) = -Tinv.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);
	return Tinv;
}

Eigen::Matrix<double, 4, 1> quatType::inv(Eigen::Matrix<double, 4, 1> q)
{
	Eigen::Matrix<double, 4, 1> q_inv;
	q_inv.block<3, 1>(0, 0) = -q.block<3, 1>(0, 0);
	q_inv(3, 0) = q(3, 0);
	return q_inv;
}

Eigen::Matrix<double, 4, 4> quatType::omega(Eigen::Matrix<double, 3, 1> w)
{
	Eigen::Matrix<double, 4, 4> mat;
	mat.block<3, 3>(0, 0) = -skewSymmetric(w);
	mat.block<1, 3>(3, 0) = -w.transpose();
	mat.block<3, 1>(0, 3) = w;
	mat(3, 3) = 0;
	return mat;
}

Eigen::Matrix<double, 4, 1> quatType::quatNorm(Eigen::Matrix<double, 4, 1> q_t)
{
	if (q_t(3, 0) < 0)
	{
		q_t *= -1;
	}

	return q_t / q_t.norm();
}

Eigen::Matrix<double, 3, 3> quatType::JleftSo3(const Eigen::Matrix<double, 3, 1> &w)
{
	double theta = w.norm();

	if (theta < 1e-6) {
		return Eigen::MatrixXd::Identity(3, 3);
	}
	else
	{
		Eigen::Matrix<double, 3, 1> a = w / theta;
		Eigen::Matrix<double, 3, 3> J = sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) + (1 - sin(theta) / theta) * a * a.transpose() +
										((1 - cos(theta)) / theta) * skewSymmetric(a);
		return J;
	}
}

Eigen::Matrix<double, 3, 3> quatType::JrighySo3(const Eigen::Matrix<double, 3, 1> &w)
{
	return JleftSo3(-w);
}

Eigen::Matrix<double, 3, 1> quatType::rotToRpy(const Eigen::Matrix<double, 3, 3> &rot)
{
	Eigen::Matrix<double, 3, 1> rpy;
	rpy(1, 0) = atan2(-rot(2, 0), sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
	if (std::abs(cos(rpy(1, 0))) > 1.0e-12)
	{
		rpy(2, 0) = atan2(rot(1, 0) / cos(rpy(1, 0)), rot(0, 0) / cos(rpy(1, 0)));
		rpy(0, 0) = atan2(rot(2, 1) / cos(rpy(1, 0)), rot(2, 2) / cos(rpy(1, 0)));
	}
	else
	{
		rpy(2, 0) = 0;
		rpy(0, 0) = atan2(rot(0, 1), rot(1, 1));
	}
	return rpy;
}

Eigen::Matrix<double, 3, 3> quatType::rotX(double t)
{
	Eigen::Matrix<double, 3, 3> r;
	double ct = cos(t);
	double st = sin(t);
	r << 1.0, 0.0, 0.0, 0.0, ct, -st, 0.0, st, ct;
	return r;
}

Eigen::Matrix<double, 3, 3> quatType::rotY(double t)
{
	Eigen::Matrix<double, 3, 3> r;
	double ct = cos(t);
	double st = sin(t);
	r << ct, 0.0, st, 0.0, 1.0, 0.0, -st, 0.0, ct;
	return r;
}

Eigen::Matrix<double, 3, 3> quatType::rotZ(double t)
{
	Eigen::Matrix<double, 3, 3> r;
	double ct = cos(t);
	double st = sin(t);
	r << ct, -st, 0.0, st, ct, 0.0, 0.0, 0.0, 1.0;
	return r;
}