#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "blh.h"
#include "insdata.h"
class InsState
{
	using v3 = Eigen::Vector3d;
	using qt = Eigen::Quaterniond;
public:
	// ctor
	InsState()
		: gpstime(0.0), pos(), vel(), att()
	{}
	InsState(double t, const BLH& p, const v3& v, const qt& a)
		: gpstime(t), pos(p), vel(v), att(a)
	{}
	InsState(const InsState& other)
		: gpstime(other.gpstime), pos(other.pos), vel(other.vel), att(other.att)
	{}
	InsState& operator=(const InsState& other);
private:
	// private function
	void TimeUpt(double t);
	void AttUpt(const InsState& prevs, const v3& currgyro, const v3& prevgyro);
	void VelUpt(const InsState& prevs, const InsState& pprevs,
		const v3& currgyro, const v3& prevgyro,	const v3& curraccl, const v3& prevaccl);
	void PosUpt(const InsState& prevs);
	InsState extrapolateTo(const InsState& prevs, const InsState& pprevs, double time);
public:
	// public function
	void Update(const InsState& prevs, const InsState& pprevs, const InsData& currd, const InsData& prevd);
	void Correct(const Eigen::Vector<double, 21>& system_vector);
public:
	double gpstime;
	BLH pos;
	v3 vel;
	qt att;
};

