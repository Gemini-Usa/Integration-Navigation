#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "blh.h"
#include "insdata.h"
class insstate
{
	using v3 = Eigen::Vector3d;
	using qt = Eigen::Quaterniond;
public:
	// ctor
	insstate()
		: gpstime(0.0), pos(), vel(), att()
	{}
	insstate(double t, const blh& p, const v3& v, const qt& a)
		: gpstime(t), pos(p), vel(v), att(a)
	{}
	insstate(const insstate& other)
		: gpstime(other.gpstime), pos(other.pos), vel(other.vel), att(other.att)
	{}
	insstate& operator=(const insstate& other);
private:
	// private function
	void TimeUpt(double t);
	void AttUpt(const insstate& prevs, const v3& currgyro, const v3& prevgyro);
	void VelUpt(const insstate& prevs, const insstate& pprevs,
		const v3& currgyro, const v3& prevgyro,	const v3& curraccl, const v3& prevaccl);
	void PosUpt(const insstate& prevs);
	insstate extrapolateTo(const insstate& prevs, const insstate& pprevs, double time);
public:
	// public function
	void Update(const insstate& prevs, const insstate& pprevs, const insdata& currd, const insdata& prevd);
	void Correct(const Eigen::Vector<double, 21>& system_vector);
public:
	double gpstime;
	blh pos;
	v3 vel;
	qt att;
};

