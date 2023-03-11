#pragma once
#include "angle.h"
#define RE 6378137.0
#define FE (1.0/298.257223563)
#define OMGE (7.2921150 * 1.0E-5)

constexpr double Tgb = 3600;
constexpr double Tab = 3600;
constexpr double Tgs = 3600;
constexpr double Tas = 3600;
constexpr double VRW = 0.4 / 60.0;
constexpr double ARW = 0.2 * Angle::D2R / 60.0;
const double std_gb = 24 * Angle::D2R / 3600.0;
const double std_ab = 4.0E-3;
const double std_gs = 1.0E-3;
const double std_as = 1.0E-3;
constexpr std::array<double, 3> lb{ 0, 0.2380, 3.3650 };
constexpr std::array<double, 3> lw{ -0.05, -0.842, 0.5 };
