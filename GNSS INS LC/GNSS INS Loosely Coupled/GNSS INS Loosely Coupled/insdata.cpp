#include "insdata.h"
using v3 = Eigen::Vector3d;
using m3 = Eigen::Matrix3d;
double InsData::getTime() const
{
    return m_gpstime;
}

v3 InsData::getGyroData() const
{
    return m_gyro;
}

v3 InsData::getAcclData() const
{
    return m_accl;
}

void InsData::Correct(const Eigen::Vector<double, 21>& system, double dt)
{
    Eigen::Vector3d bg, ba, sg, sa;
    bg = system.segment(9, 3);
    ba = system.segment(12, 3);
    sg = system.segment(15, 3);
    sa = system.segment(18, 3);
    m_gyro_bias += bg * dt;
    m_gyro_scal += sg;
    m_accl_bias += ba * dt;
    m_accl_scal += sa;
}

void InsData::Compensate(const InsData& other)
{
    m_gyro_bias = other.m_gyro_bias;
    m_gyro_scal = other.m_gyro_scal;
    m_accl_bias = other.m_accl_bias;
    m_accl_scal = other.m_accl_scal;
    // gyro
    m3 Sg = m_gyro_scal.asDiagonal().toDenseMatrix() + m3::Identity();
    m_gyro = Sg.inverse() * (m_gyro - m_gyro_bias);
    // accl
    m3 Sa = m_accl_scal.asDiagonal().toDenseMatrix() + m3::Identity();
    m_accl = Sa.inverse() * (m_accl - m_accl_bias);
}

InsData InsData::interpolateFrom(const InsData& formdata, InsData& backdata, double time)
{
    InsData middledata;
    middledata.m_gpstime = time;
    // k=(t-t1)/(t2-t1)
    double k = (time - formdata.getTime()) / (backdata.getTime() - formdata.getTime());
    // middle data
    middledata.m_gyro = k * backdata.m_gyro;
    middledata.m_accl = k * backdata.m_accl;
    // back data
    backdata.m_gyro *= 1 - k;
    backdata.m_accl *= 1 - k;
    return middledata;
}
