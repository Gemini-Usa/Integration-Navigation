#include "gnssdata.h"
using v3 = Eigen::Vector3d;
gnssdata& gnssdata::operator=(const gnssdata& other)
{
	if (!(this == &other)) {
		this->m_gpstime = other.m_gpstime;
		this->m_blh = other.m_blh;
		this->m_blhstd = other.m_blhstd;
		this->m_vel = other.m_vel;
		this->m_velstd = other.m_velstd;
	}
	return *this;
}

double gnssdata::getTime() const
{
	return m_gpstime;
}

double gnssdata::getB() const
{
	return m_blh.b.getrad();
}

double gnssdata::getL() const
{
	return m_blh.l.getrad();
}

double gnssdata::getH() const
{
	return m_blh.h;
}

double gnssdata::getVn() const
{
	return m_vel(0);
}

double gnssdata::getVe() const
{
	return m_vel(1);
}

double gnssdata::getVd() const
{
	return m_vel(2);
}

v3 gnssdata::getBLHStd() const
{
	return m_blhstd;
}

v3 gnssdata::getVelStd() const
{
	return m_velstd;
}

void gnssdata::setTime(double time)
{
	m_gpstime = time;
}

void gnssdata::setBLH(const blh& pos)
{
	m_blh = pos;
}

void gnssdata::setVel(const v3& vel)
{
	m_vel = vel;
}

void gnssdata::setBLHStd(const v3& blhstd)
{
	m_blhstd = blhstd;
}

void gnssdata::setVelStd(const v3& velstd)
{
	m_velstd = velstd;
}
