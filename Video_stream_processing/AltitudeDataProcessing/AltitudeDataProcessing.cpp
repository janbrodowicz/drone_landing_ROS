#include "AltitudeDataProcessing.h"

using namespace altitude;

int LidarAltitude::exp_circle_size(void)
{
    int circle_size;
    circle_size = int(3.43375604675693e-15 * pow(altitude_, 8) - 4.77097482299868e-12 * pow(altitude_, 7) +
                        2.79264529276849e-09 * pow(altitude_, 6) - 8.96208749876710e-07 * pow(altitude_, 5) + 0.000172002819190201 * pow(altitude_, 4) -
                        0.0202309024343306 * pow(altitude_, 3) + 1.44107699084859 * pow(altitude_, 2) - 59.7115579325702 * altitude_ + 1345.58241758463);
    
    return circle_size;
}

int PixhawkAltitude::exp_circle_size(void)
{
    int circle_size;
    circle_size = int(- 0.000110976007441677 * pow(altitude_, 3) + 0.0433422982169812 * pow(altitude_, 2) - 6.67322230364540 * altitude_ + 481.517071405531);
    
    return circle_size;
}

int DefaultAltitude::exp_circle_size(void)
{
    return altitude_;
}