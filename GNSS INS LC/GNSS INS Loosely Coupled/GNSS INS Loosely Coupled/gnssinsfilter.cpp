#include "gnssinsfilter.h"

void GnssInsFilter::setData(const InsState& is, const InsData& icurrd, const InsData& iprevd, const GnssData& gd)
{
	this->state = is;
	this->currd = icurrd;
	this->prevd = iprevd;
	this->gdata = gd;
}

double GnssInsFilter::getStdGyro() const
{
	double std = 0.0;
	for (size_t i = 9; i < 15; i++) {
		std += _P(i, i);
	}
	return sqrt(std);
}

double GnssInsFilter::getStdAccl() const
{
	double std = 0.0;
	for (size_t i = 15; i < 21; i++) {
		std += _P(i, i);
	}
	return sqrt(std);
}

void GnssInsFilter::UpdateWithAvu(int avu, const OdoData& odata)
{
	setTime();
	if (avu >= 4 && fabs(odata.getTime() - currd.getTime()) > 1.0E-9) return;
	v3 v_wheel = v3::Zero();
	m3 C_nb = state.att.toRotationMatrix().transpose();
	v3 lever{ lw[0], lw[1], lw[2] };
	v3 omg_ibb = currd.getGyroData() / _dt;
	v3 omg_nib = -C_nb * BLH::getAng_innVec(state.pos.b.getrad(), state.pos.h, state.vel(0), state.vel(1));
	v3 omg_nbb = omg_nib + omg_ibb;
	v3 hatv_wheel = C_nb * state.vel + omg_nbb.cross(lever);
	v_wheel = hatv_wheel;
	switch (avu) {
	case AVU::NHC: v_wheel(1) = 0.0; v_wheel(2) = 0.0; break;
	case AVU::ZUPT: v_wheel(0) = 0.0; v_wheel(1) = 0.0; v_wheel(2) = 0.0; break;
	case AVU::ODO: v_wheel(0) = odata.getVel(); break;
	case AVU::NHC | AVU::ODO: v_wheel(0) = odata.getVel(); v_wheel(1) = 0.0; v_wheel(2) = 0.0; break;
	default: return;
	}
	using Kdim = Eigen::Matrix<double, 21, 3>;
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Matrix<double, 21, 21> I = Eigen::Matrix<double, 21, 21>::Identity();
	Eigen::Matrix<double, 3, 21> H = Eigen::Matrix<double, 3, 21>::Zero();
	H.block(0, 3, 3, 3) = C_nb;
	H.block(0, 6, 3, 3) = -C_nb * state.vel.asSkewSymmetric();
	H.block(0, 9, 3, 3) = -lever.asSkewSymmetric().toDenseMatrix();
	v3 dz = hatv_wheel - v_wheel;
	Kdim K = _P * H.transpose() * (H * _P * H.transpose() + R).inverse();
	_dx = _dx + K * (dz - H * _dx);
	_P = (I - K * H) * _P * (I - K * H).transpose() + K * R * K.transpose();
}

void GnssInsFilter::setTime()
{
	_dt = currd.getTime() - prevd.getTime();
}

void GnssInsFilter::setF_rr(double vn, double ve, double vd, double phi, double h)
{
	double R_M = BLH::getR_M(phi);
	double R_N = BLH::getR_N(phi);
	Eigen::Matrix3d Frr = Eigen::Matrix3d::Zero();
	Frr(0, 0) = -vd / (R_M + h);
	Frr(0, 2) = vn / (R_M + h);
	Frr(1, 0) = ve * tan(phi) / (R_N + h);
	Frr(1, 1) = -(vd + vn * tan(phi)) / (R_N + h);
	Frr(1, 2) = ve / (R_N + h);
	_F.block(0, 0, 3, 3) = Frr;
}

void GnssInsFilter::setF_vr(double vn, double ve, double vd, double phi, double h)
{
	double R_M = BLH::getR_M(phi);
	double R_N = BLH::getR_N(phi);
	double gp = BLH::getG_p(phi, h);
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
	_F.block(3, 0, 3, 3) = Fvr;
}

void GnssInsFilter::setF_vv(double vn, double ve, double vd, double phi, double h)
{
	double R_M = BLH::getR_M(phi);
	double R_N = BLH::getR_N(phi);
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
	_F.block(3, 3, 3, 3) = Fvv;
}

void GnssInsFilter::setF_phir(double vn, double ve, double vd, double phi, double h)
{
	double R_M = BLH::getR_M(phi);
	double R_N = BLH::getR_N(phi);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d Fphir = Eigen::Matrix3d::Zero();
	Fphir(0, 0) = -OMGE * sinphi / (R_M + h);
	Fphir(0, 2) = ve / ((R_N + h) * (R_N + h));
	Fphir(1, 2) = -vn / ((R_M + h) * (R_M + h));
	Fphir(2, 0) = -OMGE * cosphi / (R_M + h) - ve / ((R_M + h) * (R_N + h) * cosphi * cosphi);
	Fphir(2, 2) = -ve * tanphi / ((R_N + h) * (R_N + h));
	_F.block(6, 0, 3, 3) = Fphir;
}

void GnssInsFilter::setF_phiv(double phi, double h)
{
	double R_M = BLH::getR_M(phi);
	double R_N = BLH::getR_N(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d Fphiv = Eigen::Matrix3d::Zero();
	Fphiv(0, 1) = 1 / (R_N + h);
	Fphiv(1, 0) = -1 / (R_M + h);
	Fphiv(2, 1) = -tanphi / (R_N + h);
	_F.block(6, 3, 3, 3) = Fphiv;
}

void GnssInsFilter::setF_Anginn(double vn, double ve, double phi, double h)
{
	Eigen::Matrix3d FAng_inn = Eigen::Matrix3d::Zero();
	double R_M = BLH::getR_M(phi);
	double R_N = BLH::getR_N(phi);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Vector3d Ang_inn = Eigen::Vector3d::Zero();
	Ang_inn(0) = OMGE * cosphi + ve / (R_N + h);
	Ang_inn(1) = -vn / (R_M + h);
	Ang_inn(2) = -OMGE * sinphi - ve * tanphi / (R_N + h);
	FAng_inn = Eigen::SkewSymmetricMatrix3<double>(Ang_inn(0), Ang_inn(1), Ang_inn(2));
	_F.block(6, 6, 3, 3) = -FAng_inn;
}

void GnssInsFilter::buildF()
{
	auto ang = currd.getGyroData();
	auto vel = currd.getAcclData();
	Eigen::Vector3d fb{ vel[0], vel[1], vel[2] };
	Eigen::Vector3d omg_ibb{ ang[0], ang[1], ang[2] };
	fb /= _dt, omg_ibb /= _dt;
	auto C_bn = state.att.toRotationMatrix();
	auto I_33 = Eigen::Matrix3d::Identity();

	_F.block(0, 3, 3, 3) = I_33;
	_F.block(9, 9, 3, 3) = -I_33 / Tgb;
	_F.block(12, 12, 3, 3) = -I_33 / Tab;
	_F.block(15, 15, 3, 3) = -I_33 / Tgs;
	_F.block(18, 18, 3, 3) = -I_33 / Tas;

	_F.block(6, 9, 3, 3) = -C_bn;
	_F.block(3, 12, 3, 3) = C_bn;
	_F.block(3, 6, 3, 3) = (C_bn * fb).asSkewSymmetric();
	_F.block(6, 15, 3, 3) = -C_bn * omg_ibb.asDiagonal();
	_F.block(3, 18, 3, 3) = C_bn * fb.asDiagonal();

	setF_rr(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_vr(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_vv(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_phir(state.vel[0], state.vel[1], state.vel[2], state.pos.b.getrad(), state.pos.h);
	setF_phiv(state.pos.b.getrad(), state.pos.h);
	setF_Anginn(state.vel[0], state.vel[1], state.pos.b.getrad(), state.pos.h);
}

void GnssInsFilter::buildq()
{
	_q = qdim::Zero();
	auto I_33 = Eigen::Matrix<double, 3, 3>::Identity();
	_q.block(0, 0, 3, 3) = VRW * VRW * I_33;
	_q.block(3, 3, 3, 3) = ARW * ARW * I_33;
	_q.block(6, 6, 3, 3) = 2 * std_gb * std_gb * I_33 / Tgb;
	_q.block(9, 9, 3, 3) = 2 * std_ab * std_ab * I_33 / Tab;
	_q.block(12, 12, 3, 3) = 2 * std_gs * std_gs * I_33 / Tgs;
	_q.block(15, 15, 3, 3) = 2 * std_as * std_as * I_33 / Tas;
}

void GnssInsFilter::buildG()
{
	_G = Gdim::Zero();
	Eigen::Matrix3d C_bn = state.att.toRotationMatrix();
	Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
	_G.block(3, 0, 3, 3) = C_bn;
	_G.block(6, 3, 3, 3) = C_bn;
	_G.block(9, 6, 3, 3) = I_33;
	_G.block(12, 9, 3, 3) = I_33;
	_G.block(15, 12, 3, 3) = I_33;
	_G.block(18, 15, 3, 3) = I_33;
}

void GnssInsFilter::buildQ()
{
	buildG();
	Fdim Phi = Fdim::Identity() + _F * _dt;
	_Q = (Phi * _G * _q * _G.transpose() * Phi.transpose() + _G * _q * _G.transpose()) * 0.5 * _dt;
}

void GnssInsFilter::setH_r(const quat& att)
{
	Eigen::Matrix<double, 3, 21> Hr = Eigen::Matrix<double, 3, 21>::Zero();
	Eigen::Vector3d lever{ lb[0], lb[1], lb[2] };
	Hr.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
	Hr.block(0, 6, 3, 3) = (att.toRotationMatrix() * lever).asSkewSymmetric();
	_H.block(0, 0, 3, 21) = Hr;
}

void GnssInsFilter::setH_v(const quat& att, const v3& omg_ibb, double vn, double ve, double phi, double h)
{
	Eigen::Matrix<double, 3, 21> Hv = Eigen::Matrix<double, 3, 21>::Zero();
	Eigen::Vector3d lever{ lb[0], lb[1], lb[2] };
	double R_M = BLH::getR_M(phi);
	double R_N = BLH::getR_N(phi);
	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double tanphi = tan(phi);
	Eigen::Matrix3d C_bn = att.toRotationMatrix();
	Eigen::Vector3d Ang_inn = BLH::getAng_innVec(phi, h, vn, ve);
	Eigen::Matrix3d H_v3 = -(Ang_inn.asSkewSymmetric() * (C_bn * lever).asSkewSymmetric()) - (C_bn * lever.cross(omg_ibb)).asSkewSymmetric().toDenseMatrix();
	Eigen::Matrix3d H_v6 = -C_bn * lever.asSkewSymmetric() * omg_ibb.asDiagonal();
	Hv.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
	Hv.block(0, 6, 3, 3) = H_v3;
	Hv.block(0, 9, 3, 3) = -C_bn * lever.asSkewSymmetric();
	Hv.block(0, 15, 3, 3) = H_v6;
	_H.block(3, 0, 3, 21) = Hv;
}

void GnssInsFilter::buildH()
{
	auto ang = currd.getGyroData();
	Eigen::Vector3d omg_ibb{ ang[0], ang[1], ang[2] };
	omg_ibb /= _dt;
	setH_r(state.att);
	setH_v(state.att, omg_ibb, state.vel[0], state.vel[1], state.pos.b.getrad(), state.pos.h);
}

void GnssInsFilter::buildR()
{
	using v3 = Eigen::Vector3d;
	using v6 = Eigen::Vector<double, 6>;
	v3 stdblh = gdata.getBLHStd();
	v3 stdvel = gdata.getVelStd();
	v6 R1{ pow(stdblh(0), 2), pow(stdblh(1), 2), pow(stdblh(2), 2), pow(stdvel(0), 2), pow(stdvel(1), 2), pow(stdvel(2), 2) };
	_R = R1.asDiagonal();
}

void GnssInsFilter::buildz()
{
	Eigen::Vector3d lever{ lb[0], lb[1], lb[2] };
	auto ang = currd.getGyroData();
	Eigen::Vector3d omg_ibb{ ang[0], ang[1], ang[2] };
	omg_ibb /= _dt;
	double R_M = BLH::getR_M(state.pos.b.getrad());
	double R_N = BLH::getR_N(state.pos.b.getrad());
	// position observation
	v3 r_G(gdata.getB(), gdata.getL(), gdata.getH());
	m3 invD_R = BLH::getinv_DR(state.pos.b.getrad(), state.pos.h);
	v3 hatr_I(state.pos.b.getrad(), state.pos.l.getrad(), state.pos.h);
	v3 hatr_G = hatr_I + invD_R * state.att.toRotationMatrix() * lever;
	v3 z_r = invD_R.inverse() * (hatr_G - r_G);
	// velocity observation
	v3 v_G(gdata.getVn(), gdata.getVe(), gdata.getVd());
	v3 hatv_I(state.vel(0), state.vel(1), state.vel(2));
	v3 omg_inn = BLH::getAng_ienVec(state.pos.b.getrad()) + BLH::getAng_ennVec(state.pos.b.getrad(), state.pos.h, state.vel(0), state.vel(1));
	v3 hatv_G = hatv_I - omg_inn.asSkewSymmetric() * state.att.toRotationMatrix() * lever - state.att.toRotationMatrix() * lever.cross(omg_ibb);
	v3 z_v = hatv_G - v_G;
	_dz << z_r, z_v;
}
