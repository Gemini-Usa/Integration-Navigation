#pragma once
#include <tuple>
#include "angle.h"
#include "insstate.h"
#include "gnssdata.h"
class kalmanfilter
{
	using v3 = Eigen::Vector3d;
	using v6 = Eigen::Vector<double, 6>;
	using m6 = Eigen::Matrix<double, 6, 6>;
	using v21 = Eigen::Vector<double, 21>;
	using m21 = Eigen::Matrix<double, 21, 21>;
	using Fdim = Eigen::Matrix<double, 21, 21>;
	using qdim = Eigen::Matrix<double, 18, 18>;
	using Gdim = Eigen::Matrix<double, 21, 18>;
	using Qdim = Eigen::Matrix<double, 21, 21>;
	using Hdim = Eigen::Matrix<double, 6, 21>;
	using Rdim = Eigen::Matrix<double, 6, 6>;
	using zdim = Eigen::Vector<double, 6>;
public:
	kalmanfilter()
		: m_F(Fdim::Zero()), m_q(qdim::Zero()), m_G(Gdim::Zero()), m_Q(Qdim::Zero()), 
		m_H(Hdim::Zero()), m_R(Rdim::Zero()), m_z(zdim::Zero())
	{
		buildq();// q is constant matrix
	}
	void OneStepPredict(std::tuple<v21, m21>& xandP, const insstate& istate, const insdata& currd, const insdata& prevd);
	void MeasurementUpdate(std::tuple<v21, m21>& xandP, const gnssdata& gdata, const insstate& istate, const insdata& currd, const insdata& prevd);
private:
	// one step predict
	// F
	void setF_rr(double vn, double ve, double vd, double phi, double h);
	void setF_vr(double vn, double ve, double vd, double phi, double h);
	void setF_vv(double vn, double ve, double vd, double phi, double h);
	void setF_phir(double vn, double ve, double vd, double phi, double h);
	void setF_phiv(double phi, double h);
	void setF_Anginn(double vn, double ve, double phi, double h);
	void buildF(const insstate& state, const insdata& currd, const insdata& prevd);
	void buildq();// q
	void buildG(const insstate& state);	// G
	void buildQ(double dt);				// Q
	// measurement update
	// H
	void setH_r(const Eigen::Quaterniond& att);
	void setH_v(const Eigen::Quaterniond& att, const Eigen::Vector3d& omg_ibb, double vn, double ve, double phi, double h);
	void buildH(const insstate& state, const insdata& currd, const insdata& prevd);
	void buildR(const v3& stdblh, const v3& stdvel);// R
	void buildz(const gnssdata& gstate, const insstate& istate, const insdata& idata, double dt);// z
	// clear
	void clear();
private:
	Fdim m_F;
	qdim m_q;
	Gdim m_G;
	Qdim m_Q;
	Hdim m_H;
	Rdim m_R;
	zdim m_z;
};

