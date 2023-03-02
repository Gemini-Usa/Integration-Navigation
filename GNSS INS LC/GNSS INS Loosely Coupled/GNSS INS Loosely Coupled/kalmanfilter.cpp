#include "kalmanfilter.h"
constexpr double Tgb = 3600;
constexpr double Tab = 3600;
constexpr double Tgs = 3600;
constexpr double Tas = 3600;
constexpr double VRW = 0.4 / 60.0;
constexpr double ARW = 0.2 * angle::D2R / 60.0;
const double std_gb = 24 * angle::D2R / 3600.0;
const double std_ab = 4.0E-3;
const double std_gs = 1.0E-3;
const double std_as = 1.0E-3;
constexpr std::array<double, 3> lb{ 0, 0.2380, 3.3650 };

using v3 = Eigen::Vector<double, 3>;
using m3 = Eigen::Matrix<double, 3, 3>;
using v6 = Eigen::Vector<double, 6>;
using m6 = Eigen::Matrix<double, 6, 6>;
using v21 = Eigen::Vector<double, 21>;
using m21 = Eigen::Matrix<double, 21, 21>;
using Fdim = Eigen::Matrix<double, 21, 21>;
using qdim = Eigen::Matrix<double, 18, 18>;
using Gdim = Eigen::Matrix<double, 21, 18>;
using Qdim = Eigen::Matrix<double, 21, 21>;
using Hdim = Eigen::Matrix<double, 6, 21>;

void kalmanfilter::OneStepPredict(std::tuple<v21, m21>& xandP, const insstate& state, const insdata& currd, const insdata& prevd)
{
	double dt = currd.getTime() - prevd.getTime();
	buildF(state, currd, prevd);
	buildG(state);
	buildQ(dt);
	Eigen::Matrix<double, 21, 21> Phi = Eigen::Matrix<double, 21, 21>::Identity() + m_F * dt;
	auto& [x, P] = xandP;
	x = Phi * x;
	P = Phi * P * Phi.transpose() + m_Q;
	this->clear();
}

void kalmanfilter::MeasurementUpdate(std::tuple<v21, m21>& xandP, const gnssdata& gdata, const insstate& istate, const insdata& currd, const insdata& prevd)
{
	double dt = gdata.getTime() - prevd.getTime();
	auto& [x, P] = xandP;
	Eigen::Matrix<double, 21, 21> I = Eigen::Matrix<double, 21, 21>::Identity();
	buildH(istate, currd, prevd);
	buildR(gdata.getBLHStd(), gdata.getVelStd());
	buildz(gdata, istate, currd, dt);
	Eigen::Matrix<double, 21, 6> K = P * m_H.transpose() * (m_H * P * m_H.transpose() + m_R).inverse();
	x = x + K * (m_z - m_H * x);
	P = (I - K * m_H) * P * (I - K * m_H).transpose() + K * m_R * K.transpose();
	this->clear();
}

void kalmanfilter::setF_rr(double vn, double ve, double vd, double phi, double h)
{
	double R_M = blh::getR_M(phi);
	double R_N = blh::getR_N(phi);
	Eigen::Matrix3d Frr = Eigen::Matrix3d::Zero();
	Frr(0, 0) = -vd / (R_M + h);
	Frr(0, 2) = vn / (R_M + h);
	Frr(1, 0) = ve * tan(phi) / (R_N + h);
	Frr(1, 1) = -(vd + vn * tan(phi)) / (R_N + h);
	Frr(1, 2) = ve / (R_N + h);
	m_F.block(0, 0, 3, 3) = Frr;
}

void kalmanfilter::setF_vr(double vn, double ve, double vd, double phi, double h)
{
	double R_M = blh::getR_M(phi);
	double R_N = blh::getR_N(phi);
	double gp = blh::getG_p(phi, h);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d Fvr = Eigen::Matrix3d::Zero();
	Fvr(0, 0) = -2 * ve * OMGE * cosphi / (R_M + h) - ve * ve / ((R_M + h) * (R_N + h) * cosphi * cosphi);
	Fvr(0, 2) = vn * vd / ((R_M + h) * (R_M + h)) - ve * ve * tanphi / ((R_N + h) * (R_N + h));
	Fvr(1, 0) = 2 * OMGE * (vn * cosphi - vd * sinphi) / (R_M + h) + vn * ve / ((R_M + h) * (R_N + h) * cosphi * cosphi);
	Fvr(1, 2) = (ve * vd + vn * ve * tanphi) / ((R_N + h) * (R_N + h));
	Fvr(2, 0) = 2 * OMGE * ve * sinphi / (R_M + h);
	Fvr(2, 2) = -ve * ve / ((R_N + h) * (R_N + h)) - vn * vn / ((R_M + h) * (R_M + h)) + 2 * gp / (sqrt(R_M * R_N) + h);
	m_F.block(3, 0, 3, 3) = Fvr;
}

void kalmanfilter::setF_vv(double vn, double ve, double vd, double phi, double h)
{
	double R_M = blh::getR_M(phi);
	double R_N = blh::getR_N(phi);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d Fvv = Eigen::Matrix3d::Zero();
	Fvv(0, 0) = vd / (R_M + h);
	Fvv(0, 1) = -2 * (OMGE * sinphi + ve * tanphi / (R_N + h));
	Fvv(0, 2) = vn / (R_M + h);
	Fvv(1, 0) = 2 * OMGE * sinphi + ve * tanphi / (R_N + h);
	Fvv(1, 1) = (vd + vn * tanphi) / (R_N + h);
	Fvv(1, 2) = 2 * OMGE * cosphi + ve / (R_N + h);
	Fvv(2, 0) = -2 * vn / (R_M + h);
	Fvv(2, 1) = -2 * (OMGE * cosphi + ve / (R_N + h));
	m_F.block(3, 3, 3, 3) = Fvv;
}

void kalmanfilter::setF_phir(double vn, double ve, double vd, double phi, double h)
{
	double R_M = blh::getR_M(phi);
	double R_N = blh::getR_N(phi);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d Fphir = Eigen::Matrix3d::Zero();
	Fphir(0, 0) = -OMGE * sinphi / (R_M + h);
	Fphir(0, 2) = ve / ((R_N + h) * (R_N + h));
	Fphir(1, 2) = -vn / ((R_M + h) * (R_M + h));
	Fphir(2, 0) = -OMGE * cosphi / (R_M + h) - ve / ((R_M + h) * (R_N + h) * cosphi * cosphi);
	Fphir(2, 2) = -ve * tanphi / ((R_N + h) * (R_N + h));
	m_F.block(6, 0, 3, 3) = Fphir;
}

void kalmanfilter::setF_phiv(double phi, double h)
{
	double R_M = blh::getR_M(phi);
	double R_N = blh::getR_N(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d Fphiv = Eigen::Matrix3d::Zero();
	Fphiv(0, 1) = 1 / (R_N + h);
	Fphiv(1, 0) = -1 / (R_M + h);
	Fphiv(2, 1) = -tanphi / (R_N + h);
	m_F.block(6, 3, 3, 3) = Fphiv;
}

void kalmanfilter::setF_Anginn(double vn, double ve, double phi, double h)
{
	Eigen::Matrix3d FAng_inn = Eigen::Matrix3d::Zero();
	double R_M = blh::getR_M(phi);
	double R_N = blh::getR_N(phi);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Vector3d Ang_inn = Eigen::Vector3d::Zero();
	Ang_inn(0) = OMGE * cosphi + ve / (R_N + h);
	Ang_inn(1) = -vn / (R_M + h);
	Ang_inn(2) = -OMGE * sinphi - ve * tanphi / (R_N + h);
	FAng_inn = Eigen::SkewSymmetricMatrix3<double>(Ang_inn(0), Ang_inn(1), Ang_inn(2));
	m_F.block(6, 6, 3, 3) = -FAng_inn;
}

void kalmanfilter::buildF(const insstate& state, const insdata& currd, const insdata& prevd)
{
	double dt = currd.getTime() - prevd.getTime();
	auto ang = currd.getGyroData();
	auto vel = currd.getAcclData();
	Eigen::Vector3d fb{ vel[0], vel[1], vel[2] };
	Eigen::Vector3d omg_ibb{ ang[0], ang[1], ang[2] };
	fb /= dt, omg_ibb /= dt;
	auto C_bn = state.att.toRotationMatrix();
	auto I_33 = Eigen::Matrix3d::Identity();

	m_F.block(0, 3, 3, 3) = I_33;
	m_F.block(9, 9, 3, 3) = -I_33 / Tgb;
	m_F.block(12, 12, 3, 3) = -I_33 / Tab;
	m_F.block(15, 15, 3, 3) = -I_33 / Tgs;
	m_F.block(18, 18, 3, 3) = -I_33 / Tas;

	m_F.block(6, 9, 3, 3) = -C_bn;
	m_F.block(3, 12, 3, 3) = C_bn;
	m_F.block(3, 6, 3, 3) = (C_bn * fb).asSkewSymmetric();
	m_F.block(6, 15, 3, 3) = -C_bn * omg_ibb.asDiagonal();
	m_F.block(3, 18, 3, 3) = C_bn * fb.asDiagonal();

	setF_rr(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_vr(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_vv(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_phir(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_phiv(state.pos.b.getrad(), state.pos.h);
	setF_Anginn(state.vel[0], state.vel[1], state.pos.b.getrad(), state.pos.h);
}

void kalmanfilter::buildq()
{
	auto I_33 = Eigen::Matrix<double, 3, 3>::Identity();
	m_q.block(0, 0, 3, 3) = VRW * VRW * I_33;
	m_q.block(3, 3, 3, 3) = ARW * ARW * I_33;
	m_q.block(6, 6, 3, 3) = 2 * std_gb * std_gb * I_33 / Tgb;
	m_q.block(9, 9, 3, 3) = 2 * std_ab * std_ab * I_33 / Tab;
	m_q.block(12, 12, 3, 3) = 2 * std_gs * std_gs * I_33 / Tgs;
	m_q.block(15, 15, 3, 3) = 2 * std_as * std_as * I_33 / Tas;
}

void kalmanfilter::buildG(const insstate& state)
{
	Eigen::Matrix3d C_bn = state.att.toRotationMatrix();
	Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
	m_G.block(3, 0, 3, 3) = C_bn;
	m_G.block(6, 3, 3, 3) = C_bn;
	m_G.block(9, 6, 3, 3) = I_33;
	m_G.block(12, 9, 3, 3) = I_33;
	m_G.block(15, 12, 3, 3) = I_33;
	m_G.block(18, 15, 3, 3) = I_33;
}

void kalmanfilter::buildQ(double dt)
{
	Eigen::Matrix<double, 21, 21> Phi = Eigen::Matrix<double, 21, 21>::Identity() + m_F * dt;
	m_Q = (Phi * m_G * m_q * m_G.transpose() * Phi.transpose() + m_G * m_q * m_G.transpose()) * 0.5 * dt;
}

void kalmanfilter::setH_r(const Eigen::Quaterniond& att)
{
	Eigen::Matrix<double, 3, 21> Hr = Eigen::Matrix<double, 3, 21>::Zero();
	Eigen::Vector3d lever{ lb[0], lb[1], lb[2] };
	Hr.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	Hr.block(0, 6, 3, 3) = (att.toRotationMatrix() * lever).asSkewSymmetric();
	m_H.block(0, 0, 3, 21) = Hr;
}

void kalmanfilter::setH_v(const Eigen::Quaterniond& att, const Eigen::Vector3d& omg_ibb, double vn, double ve, double phi, double h)
{
	Eigen::Matrix<double, 3, 21> Hv = Eigen::Matrix<double, 3, 21>::Zero();
	Eigen::Vector3d lever{ lb[0], lb[1], lb[2] };
	double R_M = blh::getR_M(phi);
	double R_N = blh::getR_N(phi);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d C_bn = att.toRotationMatrix();
	Eigen::Vector3d Ang_inn = blh::getAng_innVec(phi, h, vn, ve);
	Eigen::Matrix3d H_v3 = -(Ang_inn.asSkewSymmetric() * (C_bn * lever).asSkewSymmetric()) - (C_bn * lever.cross(omg_ibb)).asSkewSymmetric().toDenseMatrix();
	Eigen::Matrix3d H_v6 = -C_bn * lever.asSkewSymmetric() * omg_ibb.asDiagonal();
	Hv.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
	Hv.block(0, 6, 3, 3) = H_v3;
	Hv.block(0, 9, 3, 3) = -C_bn * lever.asSkewSymmetric();
	Hv.block(0, 15, 3, 3) = H_v6;
	m_H.block(3, 0, 3, 21) = Hv;
}

void kalmanfilter::buildH(const insstate& state, const insdata& currd, const insdata& prevd)
{
	double dt = currd.getTime() - prevd.getTime();
	auto ang = currd.getGyroData();
	Eigen::Vector3d omg_ibb{ ang[0], ang[1], ang[2] };
	omg_ibb /= dt;
	setH_r(state.att);
	setH_v(state.att, omg_ibb, state.vel[0], state.vel[1], state.pos.b.getrad(), state.pos.h);
}

void kalmanfilter::buildR(const v3& stdblh, const v3& stdvel)
{
	v6 R1{ pow(stdblh(0), 2), pow(stdblh(1), 2), pow(stdblh(2), 2), pow(stdvel(0), 2), pow(stdvel(1), 2), pow(stdvel(2), 2) };
	m_R = R1.asDiagonal();
}

void kalmanfilter::buildz(const gnssdata& gstate, const insstate& istate, const insdata& idata, double dt)
{
	Eigen::Vector3d lever{ lb[0], lb[1], lb[2] };
	auto ang = idata.getGyroData();
	Eigen::Vector3d omg_ibb{ ang[0], ang[1], ang[2] };
	omg_ibb /= dt;
	double R_M = blh::getR_M(istate.pos.b.getrad());
	double R_N = blh::getR_N(istate.pos.b.getrad());
	// position observation
	v3 r_G(gstate.getB(), gstate.getL(), gstate.getH());
	m3 invD_R = blh::getinv_DR(istate.pos.b.getrad(), istate.pos.h);
	v3 hatr_I(istate.pos.b.getrad(), istate.pos.l.getrad(), istate.pos.h);
	v3 hatr_G = hatr_I + invD_R * istate.att.toRotationMatrix() * lever;
	v3 z_r = invD_R.inverse() * (hatr_G - r_G);
	// velocity observation
	v3 v_G(gstate.getVn(), gstate.getVe(), gstate.getVd());
	v3 hatv_I(istate.vel(0), istate.vel(1), istate.vel(2));
	v3 omg_inn = blh::getAng_ienVec(istate.pos.b.getrad()) + blh::getAng_ennVec(istate.pos.b.getrad(), istate.pos.h, istate.vel(0), istate.vel(1));
	v3 hatv_G = hatv_I - omg_inn.asSkewSymmetric() * istate.att.toRotationMatrix() * lever - istate.att.toRotationMatrix() * lever.cross(omg_ibb);
	v3 z_v = hatv_G - v_G;
	m_z << z_r, z_v;
}

void kalmanfilter::clear()
{
	m_F = Fdim::Zero();
	m_G = Gdim::Zero();
	m_Q = Qdim::Zero();
	m_H = Hdim::Zero();
	m_R = Rdim::Zero();
	m_z = zdim::Zero();
}
