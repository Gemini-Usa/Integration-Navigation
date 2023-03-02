#include <iostream>
#include <format>
#include <cmath>
#include <iomanip>
#include <string>
#include <tuple>
#include <Eigen/Dense>
#include "angle.h"
#include "gnssdata.h"
#include "gnssfile.h"
#include "insdata.h"
#include "insfile.h"
#include "ododata.h"
#include "odofile.h"
#include "kalmanfilter.h"
using namespace std;
using namespace Eigen;

const double t0{ 96111.0 };
const blh pos0{ 30.5121294035_deg, 114.3553132433_deg, 25.643 };
const Vector3d vel0{ -4.779, 17.747, 0.233 };
const Vector3d att0{ 1.73447908 * angle::D2R, -0.60798289 * angle::D2R, 109.00941143 * angle::D2R };
const Vector3d blhstd0 = { pow(0.010, 2), pow(0.008, 2), pow(0.013, 2) };
const Vector3d velstd0 = { pow(0.013, 2), pow(0.010, 2), pow(0.022, 2) };
const Vector3d attstd0 = { pow(static_cast<double>(0.05 * angle::D2R), 2),
						   pow(static_cast<double>(0.05 * angle::D2R), 2),
						   pow(static_cast<double>(0.05 * angle::D2R), 2) };
const double std_gb = 24 * angle::D2R / 3600.0;
const double std_ab = 4.0E-3;
const double std_gs = 1.0E-3;
const double std_as = 1.0E-3;
const Vector3d gb_std0{ pow(std_gb, 2), pow(std_gb, 2), pow(std_gb, 2) };
const Vector3d ab_std0{ pow(std_ab, 2), pow(std_ab, 2), pow(std_ab, 2) };
const Vector3d gs_std0{ pow(std_gs, 2), pow(std_gs, 2), pow(std_gs, 2) };
const Vector3d as_std0{ pow(std_as, 2), pow(std_as, 2), pow(std_as, 2) };

void OutAnswer(const insstate& state, ostream& os = cout)
{
	ofstream of;
	string fmt = "TIME:{:10.3f}s ";
	fmt += "B:{:14.10f}deg L:{:14.10f}deg H:{:10.3f}m ";
	fmt += "VN:{:10.3f}m/s VE:{:10.3f}m/s VD:{:10.3f}m/s ";
	fmt += "ROLL:{:13.8f}deg PITCH:{:13.8f}deg YAW:{:13.8f}deg";
	string filefmt = "{:10.3f} ";
	filefmt += "{:14.10f} {:14.10f} {:10.3f} ";
	filefmt += "{:10.3f} {:10.3f} {:10.3f} ";
	filefmt += "{:13.8f} {:13.8f} {:13.8f}";
	Array3d euler_angle = angle::QuaternionToEulerAngle(state.att);
	euler_angle *= angle::R2D;
	auto args = make_format_args(
		state.gpstime,
		state.pos.b.getdeg(), state.pos.l.getdeg(), state.pos.h,
		state.vel(0), state.vel(1), state.vel(2),
		euler_angle(0), euler_angle(1), euler_angle(2));
	os << vformat(filefmt, args) << endl;
}

int main()
{
	/* test */
	/* write file */
	ofstream fp("D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/result.txt", ios::out);
	if (!fp.is_open()) return 0;
	/* read data from file */
	vector<gnssdata> gdata;
	vector<insdata> idata, preidata;
	// vector<ododata> odata;
	gnssfile f1{ "D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/GNSS实验数据.txt" };
	insfile f2{ "D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/L1.imd" };
	// odofile f3{ "D:/桌面/WHU/senior/study/组合导航/大作业/2022组合导航课程考核与数据/odo.bin" };
	f1.readGnssData(gdata);
	f2.readInsData(preidata);
	// f3.readOdometerData(odata);
	/* start integration */
	Quaterniond q_att0 = angle::EulerAngleToQuaternion(att0(0), att0(1), att0(2));
	insstate istate0(t0, pos0, vel0, q_att0);// initial state vector
	size_t i, j;
	double time;
	double gtime, itime;
	/* preprocess */
	idata.reserve(preidata.size());
	for (j = 0; j < gdata.size(); j++) {
		time = gdata[j].getTime();
		if (fabs(time - t0) < 1E-9) break;
	}
	for (i = 0; i < preidata.size(); i++) {
		itime = preidata[i].getTime();
		gtime = gdata[j].getTime();
		if (itime < t0) continue;
		if (fabs(itime - gtime) > 1E-9 && itime > gtime) {
			idata.push_back(insdata::interpolateFrom(preidata[i - 1], preidata[i], gtime));
			idata.push_back(preidata[i]);
			j++;
		}
		else {
			idata.push_back(preidata[i]);
		}
	}
	for (j = 0; j < gdata.size(); j++) {
		time = gdata[j].getTime();
		if (time > t0) break;
	}
	preidata.clear();
	/* kalman filter */
	Vector<double, 21> std0 = Vector<double, 21>::Zero();// initial variance matrix
	std0.segment(0, 3) = blhstd0;
	std0.segment(3, 3) = velstd0;
	std0.segment(6, 3) = attstd0;
	std0.segment(9, 3) = gb_std0;
	std0.segment(12, 3) = ab_std0;
	std0.segment(15, 3) = gs_std0;
	std0.segment(18, 3) = as_std0;
	Matrix<double, 21, 21> P0 = std0.asDiagonal() * std0.asDiagonal();
	kalmanfilter filter;
	insstate pprevs, prevs = istate0, currs = istate0;
	tuple<Eigen::Vector<double, 21>, Eigen::Matrix<double, 21, 21>> system = 
		make_tuple(Eigen::Vector<double, 21>::Zero(), P0);
	for (i = 1; i < idata.size(); i++) {
		OutAnswer(currs, fp);
		pprevs = prevs;
		prevs = currs;
		itime = idata[i].getTime();
		gtime = gdata[j].getTime();
		idata[i].Compensate(idata[i - 1]);
		currs.Update(prevs, pprevs, idata[i], idata[i - 1]);
		filter.OneStepPredict(system, currs, idata[i], idata[i - 1]);
		if (fabs(itime - gtime) < 1E-9) {
			filter.MeasurementUpdate(system, gdata[j], currs, idata[i], idata[i - 1]);
			idata[i].Correct(get<0>(system), idata[i].getTime() - idata[i - 1].getTime());
			currs.Correct(get<0>(system));
			get<0>(system) = Eigen::Vector<double, 21>::Zero();
			j++;
		}
	}
	fp.close();
}
