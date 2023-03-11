#pragma once
class OdoData
{
public:
	OdoData()
		: m_gpstime(0.0), m_vel(0.0), m_scal(0.0)
	{}
	OdoData(double t, double v)
		: m_gpstime(t), m_vel(v), m_scal(0.0)
	{}
	static OdoData interpolateFrom(const OdoData& formdata, const OdoData& backdata, double time);
public:
	double getTime() const;
	double getVel() const;
	double getScale() const;
private:
	double m_gpstime;
	double m_vel;
	double m_scal;
};

