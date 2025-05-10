#include "camera.h"

cameraBase::cameraBase(int width_, int height_) : width(width_), height(height_)
{

}

void cameraBase::setValue(const Eigen::MatrixXd &calib)
{
	assert(calib.rows() == 8);
	camera_values = calib;

	cv::Matx33d tempK;
	tempK(0, 0) = calib(0);
	tempK(0, 1) = 0;
	tempK(0, 2) = calib(2);
	tempK(1, 0) = 0;
	tempK(1, 1) = calib(1);
	tempK(1, 2) = calib(3);
	tempK(2, 0) = 0;
	tempK(2, 1) = 0;
	tempK(2, 2) = 1;
	camera_k_OPENCV = tempK;

	cv::Vec4d tempD;
	tempD(0) = calib(4);
	tempD(1) = calib(5);
	tempD(2) = calib(6);
	tempD(3) = calib(7);
	camera_d_OPENCV = tempD;
}

Eigen::Vector2d cameraBase::undistortD(const Eigen::Vector2d &uv_dist)
{
	Eigen::Vector2f ept_1, ept_2;
	ept_1 = uv_dist.cast<float>();
	ept_2 = undistortF(ept_1);

	return ept_2.cast<double>();
}

cv::Point2f cameraBase::undistortCV(const cv::Point2f &uv_dist)
{
	Eigen::Vector2f ept_1, ept_2;
	ept_1 << uv_dist.x, uv_dist.y;
	ept_2 = undistortF(ept_1);
	cv::Point2f pt_out;
	pt_out.x = ept_2(0);
	pt_out.y = ept_2(1);

	return pt_out;
}

Eigen::Vector2d cameraBase::distortD(const Eigen::Vector2d &uv_norm)
{
	Eigen::Vector2f ept_1, ept_2;
	ept_1 = uv_norm.cast<float>();
	ept_2 = distortF(ept_1);

	return ept_2.cast<double>();
}

cv::Point2f cameraBase::distortCV(const cv::Point2f &uv_norm)
{
	Eigen::Vector2f ept_1, ept_2;
	ept_1 << uv_norm.x, uv_norm.y;
	ept_2 = distortF(ept_1);
	cv::Point2f pt_out;
	pt_out.x = ept_2(0);
	pt_out.y = ept_2(1);

	return pt_out;
}

Eigen::MatrixXd cameraBase::getValue()
{
	return camera_values;
}

cv::Matx33d cameraBase::getK()
{
	return camera_k_OPENCV;
}

cv::Vec4d cameraBase::getD()
{
	return camera_d_OPENCV;
}

int cameraBase::w()
{
	return width;
}

int cameraBase::h()
{
	return height;
}



Eigen::Vector2f cameraEqui::undistortF(const Eigen::Vector2f &uv_dist)
{
	cv::Matx33d camK = camera_k_OPENCV;
	cv::Vec4d camD = camera_d_OPENCV;

	cv::Mat mat(1, 2, CV_32F);
	mat.at<float>(0, 0) = uv_dist(0);
	mat.at<float>(0, 1) = uv_dist(1);
	mat = mat.reshape(2);

	cv::fisheye::undistortPoints(mat, mat, camK, camD);

	Eigen::Vector2f pt_out;
	mat = mat.reshape(1);
	pt_out(0) = mat.at<float>(0, 0);
	pt_out(1) = mat.at<float>(0, 1);

	return pt_out;
}

Eigen::Vector2f cameraEqui::distortF(const Eigen::Vector2f &uv_norm)
{
    Eigen::MatrixXd cam_d = camera_values;

    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double theta = std::atan(r);
    double theta_d = theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) + cam_d(6) * std::pow(theta, 7) +
                     cam_d(7) * std::pow(theta, 9);

    double inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    double cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

    Eigen::Vector2f uv_dist;
    double x1 = uv_norm(0) * cdist;
    double y1 = uv_norm(1) * cdist;
    uv_dist(0) = (float)(cam_d(0) * x1 + cam_d(2));
    uv_dist(1) = (float)(cam_d(1) * y1 + cam_d(3));

    return uv_dist;
}

void cameraEqui::computeDistortJacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta)
{
    Eigen::MatrixXd cam_d = camera_values;

    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double theta = std::atan(r);
    double theta_d = theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) + cam_d(6) * std::pow(theta, 7) +
                     cam_d(7) * std::pow(theta, 9);

    double inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    double cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

    Eigen::Matrix<double, 2, 2> duv_dxy = Eigen::Matrix<double, 2, 2>::Zero();
    duv_dxy << cam_d(0), 0, 0, cam_d(1);

    Eigen::Matrix<double, 2, 2> dxy_dxyn = Eigen::Matrix<double, 2, 2>::Zero();
    dxy_dxyn << theta_d * inv_r, 0, 0, theta_d * inv_r;

    Eigen::Matrix<double, 2, 1> dxy_dr = Eigen::Matrix<double, 2, 1>::Zero();
    dxy_dr << -uv_norm(0) * theta_d * inv_r * inv_r, -uv_norm(1) * theta_d * inv_r * inv_r;

    Eigen::Matrix<double, 1, 2> dr_dxyn = Eigen::Matrix<double, 1, 2>::Zero();
    dr_dxyn << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    Eigen::Matrix<double, 2, 1> dxy_dthd = Eigen::Matrix<double, 2, 1>::Zero();
    dxy_dthd << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    double dthd_dth = 1 + 3 * cam_d(4) * std::pow(theta, 2) + 5 * cam_d(5) * std::pow(theta, 4) + 7 * cam_d(6) * std::pow(theta, 6) +
                      9 * cam_d(7) * std::pow(theta, 8);

    double dth_dr = 1 / (r * r + 1);

    H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
    H_dz_dzn = duv_dxy * (dxy_dxyn + (dxy_dr + dxy_dthd * dthd_dth * dth_dr) * dr_dxyn);

    double x1 = uv_norm(0) * cdist;
    double y1 = uv_norm(1) * cdist;

    H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
    H_dz_dzeta(0, 0) = x1;
    H_dz_dzeta(0, 2) = 1;
    H_dz_dzeta(0, 4) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 3);
    H_dz_dzeta(0, 5) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 5);
    H_dz_dzeta(0, 6) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 7);
    H_dz_dzeta(0, 7) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 9);
    H_dz_dzeta(1, 1) = y1;
    H_dz_dzeta(1, 3) = 1;
    H_dz_dzeta(1, 4) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 3);
    H_dz_dzeta(1, 5) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 5);
    H_dz_dzeta(1, 6) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 7);
    H_dz_dzeta(1, 7) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 9);
}



Eigen::Vector2f cameraRadtan::undistortF(const Eigen::Vector2f &uv_dist)
{
    cv::Matx33d camK = camera_k_OPENCV;
    cv::Vec4d camD = camera_d_OPENCV;

    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = uv_dist(0);
    mat.at<float>(0, 1) = uv_dist(1);
    mat = mat.reshape(2);

    cv::undistortPoints(mat, mat, camK, camD);

    Eigen::Vector2f pt_out;
    mat = mat.reshape(1);
    pt_out(0) = mat.at<float>(0, 0);
    pt_out(1) = mat.at<float>(0, 1);

    return pt_out;
}

Eigen::Vector2f cameraRadtan::distortF(const Eigen::Vector2f &uv_norm)
{
	Eigen::MatrixXd cam_d = camera_values;

	double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
	double r_2 = r * r;
	double r_4 = r_2 * r_2;
	double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * uv_norm(0) * uv_norm(1) +
	            cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
	double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * uv_norm(1) * uv_norm(1)) +
	            2 * cam_d(7) * uv_norm(0) * uv_norm(1);

	Eigen::Vector2f uv_dist;
	uv_dist(0) = (float)(cam_d(0) * x1 + cam_d(2));
	uv_dist(1) = (float)(cam_d(1) * y1 + cam_d(3));

	return uv_dist;
}

void cameraRadtan::computeDistortJacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta)
{
	Eigen::MatrixXd cam_d = camera_values;

	double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
	double r_2 = r * r;
	double r_4 = r_2 * r_2;

	H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
	double x = uv_norm(0);
	double y = uv_norm(1);
	double x_2 = uv_norm(0) * uv_norm(0);
	double y_2 = uv_norm(1) * uv_norm(1);
	double x_y = uv_norm(0) * uv_norm(1);
	H_dz_dzn(0, 0) = cam_d(0) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * x_2 + 4 * cam_d(5) * x_2 * r_2) +
	                             2 * cam_d(6) * y + (2 * cam_d(7) * x + 4 * cam_d(7) * x));
	H_dz_dzn(0, 1) = cam_d(0) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r_2 + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
	H_dz_dzn(1, 0) = cam_d(1) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r_2 + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
	H_dz_dzn(1, 1) = cam_d(1) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * y_2 + 4 * cam_d(5) * y_2 * r_2) +
	                             2 * cam_d(7) * x + (2 * cam_d(6) * y + 4 * cam_d(6) * y));

	double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * uv_norm(0) * uv_norm(1) +
	            cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
	double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * uv_norm(1) * uv_norm(1)) +
	            2 * cam_d(7) * uv_norm(0) * uv_norm(1);

	H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
	H_dz_dzeta(0, 0) = x1;
	H_dz_dzeta(0, 2) = 1;
	H_dz_dzeta(0, 4) = cam_d(0) * uv_norm(0) * r_2;
	H_dz_dzeta(0, 5) = cam_d(0) * uv_norm(0) * r_4;
	H_dz_dzeta(0, 6) = 2 * cam_d(0) * uv_norm(0) * uv_norm(1);
	H_dz_dzeta(0, 7) = cam_d(0) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
	H_dz_dzeta(1, 1) = y1;
	H_dz_dzeta(1, 3) = 1;
	H_dz_dzeta(1, 4) = cam_d(1) * uv_norm(1) * r_2;
	H_dz_dzeta(1, 5) = cam_d(1) * uv_norm(1) * r_4;
	H_dz_dzeta(1, 6) = cam_d(1) * (r_2 + 2 * uv_norm(1) * uv_norm(1));
	H_dz_dzeta(1, 7) = 2 * cam_d(1) * uv_norm(0) * uv_norm(1);
}