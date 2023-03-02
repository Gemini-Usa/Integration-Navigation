#pragma once
class ododata
{
public:
	ododata()
		: m_gpstime(0.0), m_vel(0.0)
	{}
	ododata(double t, double v)
		: m_gpstime(t), m_vel(v)
	{}
private:
	double m_gpstime;
	double m_vel;
};

