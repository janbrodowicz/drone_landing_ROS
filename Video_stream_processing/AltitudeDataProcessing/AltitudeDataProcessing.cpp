#include "AltitudeDataProcessing.h"


using namespace altitude;

LidarAltitude::LidarAltitude(double altitude) : AltitudeDataProcessingBase(altitude)
{
    m_altitude = std::max(std::min((altitude * 100 - 10), 300.0), 0.0);
};

int LidarAltitude::exp_circle_size(void)
{
    int circle_size;
    circle_size = int(3.43375604675693e-15 * pow(m_altitude, 8) - 4.77097482299868e-12 * pow(m_altitude, 7) +
                        2.79264529276849e-09 * pow(m_altitude, 6) - 8.96208749876710e-07 * pow(m_altitude, 5) + 0.000172002819190201 * pow(m_altitude, 4) -
                        0.0202309024343306 * pow(m_altitude, 3) + 1.44107699084859 * pow(m_altitude, 2) - 59.7115579325702 * m_altitude + 1345.58241758463);
    
    return circle_size;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

PixhawkAltitude::PixhawkAltitude(double altitude) : AltitudeDataProcessingBase(altitude)
{
    m_altitude = std::max(std::min(altitude * 100, 300.0), 0.0);
};

int PixhawkAltitude::exp_circle_size(void)
{
    int circle_size;
    circle_size = int(- 0.000110976007441677 * pow(m_altitude, 3) + 0.0433422982169812 * pow(m_altitude, 2) - 6.67322230364540 * m_altitude + 481.517071405531);
    
    return circle_size;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

DefaultAltitude::DefaultAltitude(double altitude) : AltitudeDataProcessingBase(altitude)
{ 

};

int DefaultAltitude::exp_circle_size(void)
{
    return m_altitude;
}