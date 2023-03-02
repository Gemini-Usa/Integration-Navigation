#pragma once
#include <Eigen/Dense>
#include "blh.h"
class gnssdata
{
	using v3 = Eigen::Vector3d;
public:
	// ctor
	gnssdata()
		: m_gpstime(0.0), m_blh(), m_blhstd(), m_vel(), m_velstd()
	{}
	gnssdata(double time, const blh& pos, const v3& posstd,	const v3& vel, const v3& velstd)
		: m_gpstime(time), m_blh(pos), m_blhstd(posstd), m_vel(vel), m_velstd(velstd)
	{}
	gnssdata(const gnssdata& other)
		: m_gpstime(other.m_gpstime), m_blh(other.m_blh), m_blhstd(other.m_blhstd), m_vel(other.m_vel), m_velstd(other.m_velstd)
	{}
	gnssdata& operator=(const gnssdata& other);
	// public function
	double getTime() const;
	double getB() const;
	double getL() const;
	double getH() const;
	double getVn() const;
	double getVe() const;
	double getVd() const;
	v3 getBLHStd() const;
	v3 getVelStd() const;

	void setTime(double time);
	void setBLH(const blh& pos);
	void setVel(const v3& vel);
	void setBLHStd(const v3& blhstd);
	void setVelStd(const v3& velstd);
private:
	double m_gpstime;
	blh m_blh;
	v3 m_blhstd;
	v3 m_vel;
	v3 m_velstd;
};

