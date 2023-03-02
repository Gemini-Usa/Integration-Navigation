#pragma once
#include <corecrt_math_defines.h>
#include <Eigen/Dense>
#include <utility>
class angle;
const angle operator""_deg(long double d);
const angle operator""_rad(long double d);
class angle
{
public:
	angle()
		: m_angle(0.0)
	{}
	explicit angle(long double a)
		: m_angle(a)
	{}
	angle(const angle& a)
		: m_angle(a.m_angle)
	{}
	static constexpr long double D2R = M_PI / 180.0;
	static constexpr long double R2D = 180.0 / M_PI;

	angle& operator=(const angle& a);
	angle operator+(const angle& a) const;
	angle operator-(const angle& a) const;
	angle operator*(const angle& a) const;
	angle operator*(double n) const;
	angle operator/(const angle& a) const;
	angle operator/(const double& n) const;
	angle& operator+=(const angle& a);
	angle& operator-=(const angle& a);
	angle& operator*=(const angle& a);
	angle& operator/=(const angle& a);

	void set(const long double val);
	long double getrad() const;
	long double getdeg() const;
	static Eigen::Quaterniond EulerAngleToQuaternion(double roll, double pitch, double yaw);
	static Eigen::Array3d QuaternionToEulerAngle(const Eigen::Quaterniond& q);
	static Eigen::Matrix3d QuaternionToMatrix(const Eigen::Quaterniond& q);
	static Eigen::Array3d MatrixToEulerAngle(const Eigen::Matrix3d& DCM);
	static void CorrectEulerAngle(Eigen::Vector3d& euler);
private:
	long double m_angle;
};

