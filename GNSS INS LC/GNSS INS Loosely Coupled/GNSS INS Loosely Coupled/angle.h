#pragma once
#include <corecrt_math_defines.h>
#include <Eigen/Dense>
#include <utility>
class Angle;
const Angle operator""_deg(long double d);
const Angle operator""_rad(long double d);
class Angle
{
public:
	Angle()
		: m_angle(0.0)
	{}
	explicit Angle(long double a)
		: m_angle(a)
	{}
	Angle(const Angle& a)
		: m_angle(a.m_angle)
	{}
	static constexpr long double D2R = M_PI / 180.0;
	static constexpr long double R2D = 180.0 / M_PI;

	Angle& operator=(const Angle& a);
	Angle operator+(const Angle& a) const;
	Angle operator-(const Angle& a) const;
	Angle operator*(const Angle& a) const;
	Angle operator*(double n) const;
	Angle operator/(const Angle& a) const;
	Angle operator/(const double& n) const;
	Angle& operator+=(const Angle& a);
	Angle& operator-=(const Angle& a);
	Angle& operator*=(const Angle& a);
	Angle& operator/=(const Angle& a);

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

