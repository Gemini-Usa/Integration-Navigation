#include "gnssdata.h"
using v3 = Eigen::Vector3d;
GnssData& GnssData::operator=(const GnssData& other)
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

double GnssData::getTime() const
{
	return m_gpstime;
}

double GnssData::getB() const
{
	return m_blh.b.getrad();
}

double GnssData::getL() const
{
	return m_blh.l.getrad();
}

double GnssData::getH() const
{
	return m_blh.h;
}

double GnssData::getVn() const
{
	return m_vel(0);
}

double GnssData::getVe() const
{
	return m_vel(1);
}

double GnssData::getVd() const
{
	return m_vel(2);
}

v3 GnssData::getBLHStd() const
{
	return m_blhstd;
}

v3 GnssData::getVelStd() const
{
	return m_velstd;
}

void GnssData::setTime(double time)
{
	m_gpstime = time;
}

void GnssData::setBLH(const BLH& pos)
{
	m_blh = pos;
}

void GnssData::setVel(const v3& vel)
{
	m_vel = vel;
}

void GnssData::setBLHStd(const v3& blhstd)
{
	m_blhstd = blhstd;
}

void GnssData::setVelStd(const v3& velstd)
{
	m_velstd = velstd;
}
