#pragma once
#include "kalmanfilter.hpp"
#include "angle.h"
#include "constant.h"
#include "insstate.h"
#include "gnssdata.h"
#include "ododata.h"

namespace AVU
{
    constexpr int NHC = 0x01;
    constexpr int ZUPT = 0x02;
    constexpr int ODO = 0x04;
}

class GnssInsFilter :
    public KalmanFilter<21, 6>
{
public:
    GnssInsFilter() = delete;
    GnssInsFilter(const xdim& init_x, const Pdim& init_P)
        : KalmanFilter(init_x, init_P), _G(Gdim::Zero())
    {
        buildq();
    }
    void setData(const InsState& is, const InsData& icurrd, const InsData& iprevd, const GnssData& gd = GnssData::GnssData());
    double getStdGyro() const;
    double getStdAccl() const;
    void UpdateWithAvu(int avu, const OdoData& odata = OdoData::OdoData());
private:
    using qdim = Eigen::Matrix<double, 18, 18>;
    using Gdim = Eigen::Matrix<double, 21, 18>;
    using quat = Eigen::Quaterniond;
    using v3 = Eigen::Vector3d;
    using m3 = Eigen::Matrix3d;
protected:
    void setTime() override;
    void setF_rr(double vn, double ve, double vd, double phi, double h);
    void setF_vr(double vn, double ve, double vd, double phi, double h);
    void setF_vv(double vn, double ve, double vd, double phi, double h);
    void setF_phir(double vn, double ve, double vd, double phi, double h);
    void setF_phiv(double phi, double h);
    void setF_Anginn(double vn, double ve, double phi, double h);
    void buildF() override;
    void buildq();
    void buildG();
    void buildQ() override;
    void setH_r(const quat& att);
    void setH_v(const quat& att, const v3& omg_ibb, double vn, double ve, double phi, double h);
    void buildH() override;
    void buildR() override;
    void buildz() override;
private:
    qdim _q;
    Gdim _G;
private:
    InsState state;
    InsData currd;
    InsData prevd;
    GnssData gdata;
};

