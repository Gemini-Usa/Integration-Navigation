#include <iostream>
#include <format>
#include <cmath>
#include <iomanip>
#include <string>
#include <functional>
#include <tuple>
#include <queue>
#include <deque>
#include <Eigen/Dense>
#include "angle.h"
#include "constant.h"
#include "gnssdata.h"
#include "gnssfile.h"
#include "insdata.h"
#include "insfile.h"
#include "ododata.h"
#include "odofile.h"
#include "zerodetector.hpp"
#include "gnssinsfilter.h"
using namespace std;
using namespace Eigen;

const double t0{ 96111.0 };// time start
const pair<double, double> tloss{ 97068.0, 97158.0 };// time GNSS loss
const BLH pos0{ 30.5121294035_deg, 114.3553132433_deg, 25.643 };// pos start
const Vector3d vel0{ -4.779, 17.747, 0.233 };// vel start
const Vector3d att0{ 1.73447908 * Angle::D2R, -0.60798289 * Angle::D2R, 109.00941143 * Angle::D2R };// att start
const Vector3d blhstd0 = { 0.010, 0.008, 0.013 };
const Vector3d velstd0 = { 0.013, 0.010, 0.022 };
const Vector3d attstd0 = { static_cast<double>(0.05 * Angle::D2R),
						   static_cast<double>(0.05 * Angle::D2R),
						   static_cast<double>(0.05 * Angle::D2R) };
const Vector3d gb_std0{ std_gb, std_gb, std_gb };
const Vector3d ab_std0{ std_ab, std_ab, std_ab };
const Vector3d gs_std0{ std_gs, std_gs, std_gs };
const Vector3d as_std0{ std_as, std_as, std_as };

void OutAnswer(const InsState& state, ostream& os = cout)
{
	string fmt = "TIME:{:10.3f}s ";
	fmt += "B:{:14.10f}deg L:{:14.10f}deg H:{:10.3f}m ";
	fmt += "VN:{:10.3f}m/s VE:{:10.3f}m/s VD:{:10.3f}m/s ";
	fmt += "ROLL:{:13.8f}deg PITCH:{:13.8f}deg YAW:{:13.8f}deg";
	string filefmt = "{:10.3f} ";
	filefmt += "{:14.10f} {:14.10f} {:10.3f} ";
	filefmt += "{:10.3f} {:10.3f} {:10.3f} ";
	filefmt += "{:13.8f} {:13.8f} {:13.8f}";
	Array3d euler_angle = Angle::QuaternionToEulerAngle(state.att);
	euler_angle *= Angle::R2D;
	auto args = make_format_args(
		state.gpstime,
		state.pos.b.getdeg(), state.pos.l.getdeg(), state.pos.h,
		state.vel(0), state.vel(1), state.vel(2),
		euler_angle(0), euler_angle(1), euler_angle(2));
	os << vformat(filefmt, args) << endl;
}

void OutGyroData(double time, const Vector3d& gyro, ostream& os = cout)
{
	string filefmt = "{:10.3f} {:10.3e} {:10.3e} {:10.3e}\n";
	auto args = make_format_args(time, gyro(0), gyro(1), gyro(2));
	os << vformat(filefmt, args);
}

void OutVariance(double time, const Matrix<double, 21, 21>& P, ostream& os = cout)
{
	string filefmt = "{:10.3f} {:10.3e} {:10.3e} {:10.3e}\n";
	auto args = make_format_args(time, P(0, 0), P(3, 3), P(6, 6));
	os << vformat(filefmt, args);
}

int main()
{
	/* test */
	/* write file */
	ofstream fp("D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/result loss 120s ODO.txt", ios::out);
	if (!fp.is_open()) return 0;
	/* read data from file */
	vector<GnssData> gdata;
	vector<InsData> idata, preidata;
	vector<OdoData> odata, preodata;
	GnssFile f1{ "D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/GNSS实验数据.txt" };
	InsFile f2{ "D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/L1.imd" };
	OdoFile f3{ "D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/odo.bin" };
	f1.readGnssData(gdata);
	f2.readInsData(preidata);
	f3.readOdometerData(preodata);
	/* start integration */
	Quaterniond q_att0 = Angle::EulerAngleToQuaternion(att0(0), att0(1), att0(2));
	InsState istate0(t0, pos0, vel0, q_att0);// initial state vector
	size_t i, j, k;// subscript for Gnss, Ins, Odo respectively
	double gtime, itime, otime;
	/* preprocess */
	idata.reserve(preidata.size());
	odata.reserve(preodata.size());
	for (j = 0; j < gdata.size(); j++) {
		gtime = gdata[j].getTime();
		if (fabs(gtime - t0) < 1E-9) break;
	}
	for (i = 0; i < preidata.size(); i++) {
		itime = preidata[i].getTime();
		gtime = gdata[j].getTime();
		if (itime < t0) continue;
		if (fabs(itime - gtime) > 1E-9 && itime > gtime) {
			idata.push_back(InsData::interpolateFrom(preidata[i - 1], preidata[i], gtime));
			idata.push_back(preidata[i]);
			j++;
		}
		else {
			idata.push_back(preidata[i]);
		}
	}
	j = 1;
	for (k = 0; k < preodata.size(); k++) {
		otime = preodata[k].getTime();
		gtime = gdata[j].getTime();
		if (otime < t0) continue;
		if (fabs(otime - gtime) > 1E-9 && otime > gtime) {
			odata.push_back(OdoData::interpolateFrom(preodata[k - 1], preodata[k], gtime));
			odata.push_back(preodata[k]);
			j++;
		}
		else {
			odata.push_back(preodata[k]);
		}
	}
	for (j = 0; j < gdata.size(); j++) {
		gtime = gdata[j].getTime();
		if (gtime > t0) break;
	}
	for (k = 0; k < preodata.size(); k++) {
		otime = preodata[k].getTime();
		if (otime < t0) continue;
		odata.push_back(preodata[k]);
	}
	k = 0;
	preidata.clear();
	preodata.clear();
	/* kalman filter */
	Vector<double, 21> std0 = Vector<double, 21>::Zero();// initial_P
	std0.segment(0, 3) = blhstd0;
	std0.segment(3, 3) = velstd0;
	std0.segment(6, 3) = attstd0;
	std0.segment(9, 3) = gb_std0;
	std0.segment(12, 3) = ab_std0;
	std0.segment(15, 3) = gs_std0;
	std0.segment(18, 3) = as_std0;
	Matrix<double, 21, 21> P0 = std0.asDiagonal() * std0.asDiagonal();
	GnssInsFilter filter1(Vector<double, 21>::Zero(), P0);// GNSS/INS filter
	InsState pprevs, prevs = istate0, currs = istate0;
	AngularRateEnergyDetector are(20, idata.size(), 3.5E-5);// zero detector
	for (i = 1; i < idata.size(); i++) {
		OutAnswer(currs, fp);
		pprevs = prevs;
		prevs = currs;
		itime = idata[i].getTime();
		gtime = gdata[j].getTime();
		otime = odata[k].getTime();
		idata[i].Compensate(idata[i - 1]);
		currs.Update(prevs, pprevs, idata[i], idata[i - 1]);
		filter1.setData(currs, idata[i], idata[i - 1]);
		filter1.Predict();
		filter1.clear();
		are.windowMoveOn(make_tuple(idata[i], filter1.getTime()));
		if (are.detect()) {
			filter1.UpdateWithAvu(AVU::ZUPT);
			currs.Correct(filter1.getState());
			idata[i].Correct(filter1.getState(), filter1.getTime());
			filter1.clear();
		}
		else {
			filter1.UpdateWithAvu(AVU::ODO, odata[k++]);
			currs.Correct(filter1.getState());
			idata[i].Correct(filter1.getState(), filter1.getTime());
			filter1.clear();
		}
		if (fabs(itime - gtime) < 1E-9) {
			if (gtime > tloss.first && gtime < tloss.second) continue;// gnss loss simulation
			filter1.setData(currs, idata[i], idata[i - 1], gdata[j]);
			filter1.Update();
			currs.Correct(filter1.getState());
			idata[i].Correct(filter1.getState(), filter1.getTime());
			filter1.clear();
			j++;
		}
	}
	fp.close();
}
