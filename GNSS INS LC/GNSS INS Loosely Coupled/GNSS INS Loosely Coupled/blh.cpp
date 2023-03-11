#include "blh.h"
using v3 = Eigen::Vector3d;
using m3 = Eigen::Matrix3d;
BLH& BLH::operator=(const BLH& other)
{
	if (!(this == &other)) {
		this->b = other.b;
		this->l = other.l;
		this->h = other.h;
	}
	return *this;
}

BLH& BLH::operator-=(const BLH& other)
{
	b -= other.b;
	l -= other.l;
	h -= other.h;
	return *this;
}

BLH BLH::operator-(const BLH& other) const
{
	BLH res;
	res.b = b - other.b;
	res.l = l - other.l;
	res.h = h - other.h;
	return res;
}

BLH BLH::operator+(const BLH& other) const
{
	BLH res;
	res.b = b + other.b;
	res.l = l + other.l;
	res.h = h + other.h;
	return res;
}

BLH BLH::operator*(double scalar) const
{
	BLH res;
	res.b = b * scalar;
	res.l = l * scalar;
	res.h = h * scalar;
	return res;
}

double BLH::getR_M(double phi)
{
	double sqsinphi = sin(phi) * sin(phi);
	double e2 = FE * (2.0 - FE);
	return (RE * (1 - e2)) / (sqrt(pow((1 - e2 * sqsinphi), 3)));
}

double BLH::getR_N(double phi)
{
	double sqsinphi = sin(phi) * sin(phi);
	double e2 = FE * (2.0 - FE);
	return (RE) / (sqrt(1 - e2 * sqsinphi));
}

double BLH::getG_p(double phi, double h)
{
	double g0 = (9.7803267715 * (1.0 + 0.0052790414 * sin(phi) * sin(phi) + 0.0000232718 * pow(sin(phi), 4)));
	return g0 - (3.087691089E-6 - 4.397731E-9 * pow(sin(phi), 2)) * h + 0.721E-12 * h * h;
}

v3 BLH::getAng_ienVec(double phi)
{
	v3 Ang_ien;
	Ang_ien << OMGE * cos(phi), 0, -OMGE * sin(phi);
	return Ang_ien;
}

v3 BLH::getAng_ennVec(double phi, double h, double vn, double ve)
{
	v3 Ang_enn;
	double R_N = getR_N(phi);
	double R_M = getR_M(phi);
	Ang_enn << ve / (R_N + h), -vn / (R_M + h), -ve * tan(phi) / (R_N + h);
	return Ang_enn;
}

v3 BLH::getAng_innVec(double phi, double h, double vn, double ve)
{
	v3 Ang_ien = getAng_ienVec(phi);
	v3 Ang_enn = getAng_ennVec(phi, h, vn, ve);
	return Ang_ien + Ang_enn;
}

m3 BLH::getinv_DR(double phi, double h)
{
	double R_M = getR_M(phi);
	double R_N = getR_N(phi);
	return v3(1 / (R_M + h), 1 / ((R_N + h) * cos(phi)), -1).asDiagonal();
}

std::array<double, 3> BLH::toxyz() const
{
	std::array<double, 3> xyz;
	double e2 = 2 * FE - FE * FE;
	double R_N = getR_N(b.getrad());
	double sinb = sin(b.getrad());
	double cosb = cos(b.getrad());
	double sinl = sin(l.getrad());
	double cosl = cos(l.getrad());
	xyz[0] = (R_N + h) * cosb * cosl;
	xyz[1] = (R_N + h) * cosb * sinl;
	xyz[2] = (R_N * (1 - e2) + h) * sinb;
	return xyz;
}

std::array<double, 3> BLH::getned(const BLH& base) const
{
	std::array<double, 3> ned;
	double sinb = sin(b.getrad());
	double cosb = cos(b.getrad());
	double sinl = sin(l.getrad());
	double cosl = cos(l.getrad());
	Eigen::Matrix3d E;
	Eigen::Vector3d dr, pos;
	E << -sinl, cosl, 0,
		 -sinb * cosl, -sinb * sinl, cosb,
		  cosb * cosl,  cosb * sinl, sinb;
	auto xyz = this->toxyz();
	auto xyzb = base.toxyz();
	dr << xyz[0] - xyzb[0], xyz[1] - xyzb[1], xyz[2] - xyzb[2];
	pos = E * dr;
	for (size_t i = 0; i < 3; i++)
	{
		ned[i] = pos(i);
	}
	return ned;
}
