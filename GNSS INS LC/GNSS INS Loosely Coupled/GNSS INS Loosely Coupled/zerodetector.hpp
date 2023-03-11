#pragma once
#include <cmath>
#include <deque>
#include <tuple>
#include <Eigen/Dense>
#include "insdata.h"
template<class T>
class ZeroDetector
{
public:
	ZeroDetector() = delete;
	ZeroDetector(int window, int maxlen, double thres)
		: m_window(window), m_maxlen(maxlen), m_thres(thres)
	{
		if (window > maxlen) m_window = m_maxlen;
	}
	double getWindow() const;
	void windowMoveOn(const T& data);
	bool detect() const;
	virtual double getDetector() const = 0;
protected:
	int m_window;
	int m_maxlen;
	double m_thres;
	std::deque<T> m_detector;
};

class AngularRateEnergyDetector :
	public ZeroDetector<std::tuple<InsData, double>>// data, dt
{
	using ZeroDetector::ZeroDetector;
public:
	double getDetector() const override;
};

class AccelerationMagnitudeDetector :
	public ZeroDetector<std::tuple<InsData, double, double>>// data, g, sig, dt
{
	using ZeroDetector::ZeroDetector;
public:
	double getDetector() const override;
};

class AccelerationMovingVarianceDetector :
	public ZeroDetector<std::tuple<InsData, double>>// data, sig, dt
{
	using ZeroDetector::ZeroDetector;
public:
	double getDetector() const override;
};

class StanceHypothesisOptimalDetector :
	public ZeroDetector<std::tuple<InsData, double, double>>// data, g, dt
{
	using ZeroDetector::ZeroDetector;
public:
	double getDetector() const override;
};

template<class T>
inline double ZeroDetector<T>::getWindow() const
{
	return m_window;
}

template<class T>
inline void ZeroDetector<T>::windowMoveOn(const T& data)
{
	if (m_detector.size() < m_window) {
		m_detector.push_back(data);
	}
	else {
		m_detector.pop_front();
		m_detector.push_back(data);
	}
}

template<class T>
inline bool ZeroDetector<T>::detect() const
{
	if (this->getDetector() < m_thres) return true;
	else return false;
}

inline double AngularRateEnergyDetector::getDetector() const
{
	double sum = 0.0;
	for (const auto& [ddata, dt] : m_detector) {
		sum += pow(ddata.getGyroData().norm() / dt, 2);
	}
	return sum / m_detector.size();
}

inline double AccelerationMagnitudeDetector::getDetector() const
{
	double sum = 0.0;
	for (const auto& [ddata, g, dt] : m_detector) {
		sum += pow(ddata.getAcclData().norm() / dt - g, 2);
	}
	return sum / m_detector.size();
}

inline double AccelerationMovingVarianceDetector::getDetector() const
{
	double sum = 0.0;
	Eigen::Vector3d acc = Eigen::Vector3d::Zero();
	for (const auto& [ddata, dt] : m_detector) {
		acc += ddata.getAcclData() / dt;
	}
	acc /= m_detector.size();
	for (const auto& [ddata, dt] : m_detector) {
		sum += pow((ddata.getAcclData() / dt - acc).norm(), 2);
	}
	return sum / m_detector.size();
}

inline double StanceHypothesisOptimalDetector::getDetector() const
{
	double sum = 0.0;
	Eigen::Vector3d acc = Eigen::Vector3d::Zero();
	for (const auto& [ddata, g, dt] : m_detector) {
		acc += ddata.getAcclData() / dt;
	}
	acc /= m_detector.size();
	acc.normalize();
	for (const auto& [ddata, g, dt] : m_detector) {
		sum += pow((ddata.getAcclData() / dt - g * acc).norm() / 0.02, 2) + pow((ddata.getGyroData() / dt).norm() / (0.1 * Angle::D2R), 2);
	}
	return sum / m_detector.size();
}
