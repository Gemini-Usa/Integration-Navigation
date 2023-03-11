#pragma once
#include <Eigen/Dense>
class InsData
{
	using v3 = Eigen::Vector3d;
public:
	// ctor
	InsData()
		: m_gpstime(0.0), m_gyro(v3::Zero()), m_accl(v3::Zero()),
		m_gyro_bias(v3::Zero()), m_gyro_scal(v3::Zero()), m_accl_bias(v3::Zero()), m_accl_scal(v3::Zero())
	{}
	InsData(double time, double gx, double gy, double gz, double ax, double ay, double az)
		: m_gpstime(time), m_gyro(gx, gy, gz), m_accl(ax, ay, az),
		m_gyro_bias(v3::Zero()), m_gyro_scal(v3::Zero()), m_accl_bias(v3::Zero()), m_accl_scal(v3::Zero())
	{}
	InsData(const InsData& o)
		: m_gpstime(o.m_gpstime),
		m_gyro(o.m_gyro), m_accl(o.m_accl),
		m_gyro_bias(o.m_gyro_bias), m_gyro_scal(o.m_gyro_scal),
		m_accl_bias(o.m_accl_bias), m_accl_scal(o.m_accl_scal)
	{}
	// public function
	double getTime() const;
	v3 getGyroData() const;
	v3 getAcclData() const;
	void Correct(const Eigen::Vector<double, 21>& system, double dt);
	void Compensate(const InsData& other);
	static InsData interpolateFrom(const InsData& formdata, InsData& backdata, double time);
private:
	double m_gpstime;
	v3 m_gyro;
	v3 m_accl;
	v3 m_gyro_bias;
	v3 m_gyro_scal;
	v3 m_accl_bias;
	v3 m_accl_scal;
};

