#include "angle.h"
angle& angle::operator=(const angle& a)
{
	if (this != &a) {
		this->m_angle = a.m_angle;
	}
	return *this;
}

angle angle::operator+(const angle& a) const
{
	return angle(this->m_angle + a.m_angle);
}

angle angle::operator-(const angle& a) const
{
	return angle(this->m_angle - a.m_angle);
}

angle angle::operator*(const angle& a) const
{
	return angle(this->m_angle * a.m_angle);
}

angle angle::operator*(double n) const
{
	return angle(this->m_angle * n);
}

angle angle::operator/(const angle& a) const
{
	return angle(this->m_angle / a.m_angle);
}

angle angle::operator/(const double& n) const
{
	return angle(this->m_angle / n);
}

angle& angle::operator+=(const angle& a)
{
	this->m_angle += a.m_angle;
	return *this;
}

angle& angle::operator-=(const angle& a)
{
	this->m_angle -= a.m_angle;
	return *this;
}

angle& angle::operator*=(const angle& a)
{
	this->m_angle *= a.m_angle;
	return *this;
}

angle& angle::operator/=(const angle& a)
{
	this->m_angle /= a.m_angle;
	return *this;
}

void angle::set(const long double val)
{
	this->m_angle = val;
}

long double angle::getrad() const
{
	return this->m_angle;
}

long double angle::getdeg() const
{
	return this->m_angle * angle::R2D;
}

Eigen::Quaterniond angle::EulerAngleToQuaternion(double roll, double pitch, double yaw)
{
	double sinpsi = sin(yaw / 2), cospsi = cos(yaw / 2);
	double sinthe = sin(pitch / 2), costhe = cos(pitch / 2);
	double sinphi = sin(roll / 2), cosphi = cos(roll / 2);
	Eigen::Quaterniond q(cosphi * costhe * cospsi + sinphi * sinthe * sinpsi,
		sinphi * costhe * cospsi - cosphi * sinthe * sinpsi,
		cosphi * sinthe * cospsi + sinphi * costhe * sinpsi,
		cosphi * costhe * sinpsi - sinphi * sinthe * cospsi);
	return q;
}

Eigen::Array3d angle::QuaternionToEulerAngle(const Eigen::Quaterniond& q)
{
	// in format of roll, pitch, yaw
	Eigen::Array3d angles;

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
	angles(0) = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
	double cosp = std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
	angles(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
	angles(2) = std::atan2(siny_cosp, cosy_cosp);
	if (angles(2) < 0) angles(2) += 2 * M_PI;
	if (angles(2) > 2 * M_PI) angles(2) -= 2 * M_PI;

	return angles;
}

Eigen::Matrix3d angle::QuaternionToMatrix(const Eigen::Quaterniond& q)
{
	double q0 = q.w();
	double q1 = q.x();
	double q2 = q.y();
	double q3 = q.z();
	Eigen::Matrix3d res;
	res << q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2),
		2 * (q1 * q2 + q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2 * (q2 * q3 - q0 * q1),
		2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	return res;
}

Eigen::Array3d angle::MatrixToEulerAngle(const Eigen::Matrix3d& DCM)
{
	// in format of roll, pitch, yaw
	double pitch = atan2(-DCM(2, 0), sqrt(DCM(2, 1) * DCM(2, 1) + DCM(2, 2) * DCM(2, 2)));
	double yaw = atan2(DCM(1, 0), DCM(0, 0));
	double roll = atan2(DCM(2, 1), DCM(2, 2));
	return Eigen::Array3d(roll, pitch, yaw);
}

void angle::CorrectEulerAngle(Eigen::Vector3d& euler)
{
	double& roll = euler(2);
	double& pitch = euler(1);
	double& yaw = euler(0);
	while (roll < -M_PI) roll += M_PI;
	while (roll > M_PI) roll -= M_PI;
	while (pitch < -M_PI / 2.0) pitch += M_PI;
	while (pitch > M_PI / 2.0) pitch -= M_PI;
	while (yaw < 0) yaw += 2 * M_PI;
	while (yaw > 2 * M_PI) yaw -= 2 * M_PI;
}

const angle operator""_deg(long double d)
{
	return angle(d * angle::D2R);
}

const angle operator""_rad(long double d)
{
	return angle(d);
}

