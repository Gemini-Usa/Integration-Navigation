#include "ododata.h"

OdoData OdoData::interpolateFrom(const OdoData& formdata, const OdoData& backdata, double time)
{
    OdoData middledata;
    middledata.m_gpstime = time;
    double k = (time - formdata.getTime()) / (backdata.getTime() - formdata.getTime());
    middledata.m_vel = formdata.m_vel + k * (backdata.m_vel - formdata.m_vel);
    return middledata;
}

double OdoData::getTime() const
{
    return m_gpstime;
}

double OdoData::getVel() const
{
    return m_vel;
}

double OdoData::getScale() const
{
    return m_scal;
}
