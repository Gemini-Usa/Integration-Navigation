#pragma once
#include <cmath>
#include <array>
#include <Eigen/Dense>
#include "constant.h"
#include "angle.h"
class blh
{
	using v3 = Eigen::Vector3d;
	using m3 = Eigen::Matrix3d;
public:
	blh()
		: b(0.0), l(0.0), h(0.0)
	{}
	blh(angle b, angle l, double h)
		: b(b), l(l), h(h)
	{}
	blh(const blh& other)
		: b(other.b), l(other.l), h(other.h)
	{}
	blh& operator=(const blh& other);
	blh& operator-= (const blh& other);
	blh operator-(const blh& other) const;
	blh operator+(const blh& other) const;
	blh operator*(double scalar) const;
	static double getR_M(double phi);
	static double getR_N(double phi);
	static double getG_p(double phi, double h);
	static v3 getAng_ienVec(double phi);
	static v3 getAng_ennVec(double phi, double h, double vn, double ve);
	static v3 getAng_innVec(double phi, double h, double vn, double ve);
	static m3 getinv_DR(double phi, double h);
	std::array<double, 3> toxyz() const;
	std::array<double, 3> getned(const blh& base) const;
public:
	angle b;
	angle l;
	double h;
};

