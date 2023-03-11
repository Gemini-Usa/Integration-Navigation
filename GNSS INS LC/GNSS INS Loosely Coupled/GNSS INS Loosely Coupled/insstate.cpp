#include "insstate.h"
using v3 = Eigen::Vector3d;
using m3 = Eigen::Matrix3d;
using qt = Eigen::Quaterniond;
InsState& InsState::operator=(const InsState& other)
{
	if (this != &other) {
		gpstime = other.gpstime;
		pos = other.pos;
		vel = other.vel;
		att = other.att;
	}
	return *this;
}
void InsState::TimeUpt(double t)
{
	gpstime = t;
}

void InsState::AttUpt(const InsState& prevs, const v3& currgyro, const v3& prevgyro)
{
	double real;
	v3 img;
	double t = gpstime - prevs.gpstime;
	v3 Ang_ien = BLH::getAng_ienVec(prevs.pos.b.getrad());
	v3 Ang_enn = BLH::getAng_ennVec(prevs.pos.b.getrad(), prevs.pos.h, prevs.vel[0], prevs.vel[1]);
	v3 phik = currgyro + prevgyro.cross(currgyro) / 12.0;
	// TODO: Rotation Vector to Quaternion
	real = cos((0.5 * phik).norm());
	img = phik * (sin((0.5 * phik).norm())) / phik.norm();
	qt qb(real, img(0), img(1), img(2));
	v3 zeta = (Ang_ien + Ang_enn) * t;
	real = cos((0.5 * zeta).norm());
	img = -0.5 * zeta * (sin((0.5 * zeta).norm())) / (0.5 * zeta).norm();
	qt qn(real, img(0), img(1), img(2));
	att = (qn * prevs.att * qb).normalized();
}

void InsState::VelUpt(const InsState& prevs, const InsState& pprevs,
	const v3& currgyro, const v3& prevgyro, const v3& curraccl, const v3& prevaccl)
{
	double t = gpstime - prevs.gpstime;
	double middlet = gpstime - t / 2;
	InsState middles = extrapolateTo(prevs, pprevs, middlet);
	v3 g_pn{ 0, 0, BLH::getG_p(middles.pos.b.getrad(), middles.pos.h) };
	v3 omg_ien = BLH::getAng_ienVec(middles.pos.b.getrad());
	v3 omg_enn = BLH::getAng_ennVec(middles.pos.b.getrad(), middles.pos.h, middles.vel(0), middles.vel(1));
	v3 a_gc = g_pn - (2 * omg_ien + omg_enn).cross(middles.vel);
	v3 dv_gn = a_gc * t;
	v3 zeta = (omg_ien + omg_enn) * t;
	v3 dv_fb = curraccl + currgyro.cross(curraccl) / 2 + (prevgyro.cross(curraccl) + prevaccl.cross(currgyro)) / 12;
	v3 dv_fn = (m3::Identity() - zeta.asSkewSymmetric().toDenseMatrix() * 0.5) * prevs.att.toRotationMatrix() * dv_fb;
	vel = prevs.vel + dv_fn + dv_gn;
}

void InsState::PosUpt(const InsState& prevs)
{
	double R_M = BLH::getR_M(prevs.pos.b.getrad());
	double R_N = BLH::getR_N(prevs.pos.b.getrad());
	double t = gpstime - prevs.gpstime;
	double avg_veln = (vel(0) + prevs.vel(0)) / 2.0;
	double avg_vele = (vel(1) + prevs.vel(1)) / 2.0;
	double avg_veld = (vel(2) + prevs.vel(2)) / 2.0;
	pos.h = prevs.pos.h - avg_veld * t;
	double avg_h = (pos.h - prevs.pos.h) / 2.0;
	pos.b = prevs.pos.b + Angle(avg_veln * t / (R_M + avg_h));
	double avg_b = (pos.b + prevs.pos.b).getrad() / 2.0;
	pos.l = prevs.pos.l + Angle(avg_vele * t / ((R_N + avg_h) * cos(avg_b)));
}

InsState InsState::extrapolateTo(const InsState& prevs, const InsState& pprevs, double time)
{
	InsState state;
	state.gpstime = time;
	state.pos = prevs.pos * (3.0 / 2.0) - pprevs.pos * 0.5;
	state.vel = prevs.vel * (3.0 / 2.0) - pprevs.vel * 0.5;
	// no need for attitude
	return state;
}

void InsState::Update(const InsState& prevs, const InsState& pprevs, const InsData& currd, const InsData& prevd)
{
	TimeUpt(currd.getTime());
	AttUpt(prevs, currd.getGyroData(), prevd.getGyroData());
	VelUpt(prevs, pprevs, currd.getGyroData(), prevd.getGyroData(), currd.getAcclData(), prevd.getAcclData());
	PosUpt(prevs);
}

void InsState::Correct(const Eigen::Vector<double, 21>& system_vector)
{
	v3 dr, dv, dphi;
	double R_M = BLH::getR_M(pos.b.getrad());
	double R_N = BLH::getR_N(pos.b.getrad());
	dr = system_vector.segment(0, 3);// ned form
	dv = system_vector.segment(3, 3);
	dphi = system_vector.segment(6, 3);// angleaxis form
	m3 invD_R = v3(1 / (R_M + pos.h), 1 / ((R_N + pos.h) * cos(pos.b.getrad())), -1).asDiagonal();
	// position
	v3 dp = invD_R * dr;
	pos -= BLH(Angle(dp(0)), Angle(dp(1)), dp(2));
	// velocity
	vel -= dv;
	// attitude, TODO: Rotation Vector to Quaternion
	v3 halfphi = dphi * 0.5;
	double real = cos(halfphi.norm());
	v3 img = halfphi.normalized() * sin(halfphi.norm());
	qt dq(real, img(0), img(1), img(2));
	att = dq * att;
}
