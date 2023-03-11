#pragma once
#include <Eigen/Dense>
#include <functional>

template<size_t State, size_t Measure>
class KalmanFilter
{
protected:
	using xdim = Eigen::Vector<double, State>;
	using Pdim = Eigen::Matrix<double, State, State>;
	using Fdim = Eigen::Matrix<double, State, State>;
	using Qdim = Eigen::Matrix<double, State, State>;
	using zdim = Eigen::Vector<double, Measure>;
	using Hdim = Eigen::Matrix<double, Measure, State>;
	using Rdim = Eigen::Matrix<double, Measure, Measure>;
public:
	KalmanFilter() = delete;
	KalmanFilter(const xdim& init_x, const Pdim& init_P)
		: _dt(0.0), _dx(init_x), _P(init_P),
		_F(Fdim::Zero()),
		_Q(Qdim::Zero()),
		_dz(zdim::Zero()),
		_H(Hdim::Zero()),
		_R(Rdim::Zero())
	{}
protected:
	virtual void setTime() = 0;
	virtual void buildF() = 0;
	virtual void buildQ() = 0;
	virtual void buildz() = 0;
	virtual void buildH() = 0;
	virtual void buildR() = 0;
public:
	void Predict();
	void Update();
	void clear();
	xdim getState() const;
	double getTime() const;
protected:
	double _dt;
	xdim _dx;
	Pdim _P;
	Fdim _F;
	Qdim _Q;
	zdim _dz;
	Hdim _H;
	Rdim _R;
};

template<size_t State, size_t Measure>
inline void KalmanFilter<State, Measure>::Predict()
{
	Fdim I = Fdim::Identity();
	setTime();
	buildF();
	buildQ();
	Fdim Phi = I + _F * _dt;
	_dx = Phi * _dx;
	_P = Phi * _P * Phi.transpose() + _Q;
}

template<size_t State, size_t Measure>
inline void KalmanFilter<State, Measure>::Update()
{
	using Kdim = Eigen::Matrix<double, State, Measure>;
	Eigen::Matrix<double, State, State> I = Eigen::Matrix<double, State, State>::Identity();
	setTime();
	buildH();
	buildR();
	buildz();
	Kdim K = _P * _H.transpose() * (_H * _P * _H.transpose() + _R).inverse();
	_dx = _dx + K * (_dz - _H * _dx);
	_P = (I - K * _H) * _P * (I - K * _H).transpose() + K * _R * K.transpose();
}

template<size_t State, size_t Measure>
inline void KalmanFilter<State, Measure>::clear()
{
	_dx = xdim::Zero();
	_F = Fdim::Zero();
	_Q = Qdim::Zero();
	_dz = zdim::Zero();
	_H = Hdim::Zero();
	_R = Rdim::Zero();
}

template<size_t State, size_t Measure>
inline Eigen::Vector<double, State> KalmanFilter<State, Measure>::getState() const
{
	return _dx;
}

template<size_t State, size_t Measure>
inline double KalmanFilter<State, Measure>::getTime() const
{
	return _dt;
}
