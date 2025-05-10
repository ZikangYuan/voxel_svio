#include "mapPoint.h"

mapPoint::mapPoint() : vec(3)
{

}

void mapPoint::update(const Eigen::VectorXd &dx)
{
	assert(dx.rows() == size);
	setValue(value_ + dx);
}

Eigen::Matrix<double, 3, 1> mapPoint::getPointXYZ(bool get_fej)
{
	return (get_fej) ? fej() : value();
}

void mapPoint::setPointXYZ(Eigen::Matrix<double, 3, 1> p_FinG, bool is_fej)
{
	if (is_fej)
		setFej(p_FinG);
	else
		setValue(p_FinG);
	
	return;
}