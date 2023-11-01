#include <cmath>
#include <algorithm>
#include "AltitudeDataProcessingBase.h"

namespace altitude
{

class DefaultAltitude : public AltitudeDataProcessingBase
{
    public:

        DefaultAltitude(double altitude);

        int exp_circle_size(void) override;

};

////////////////////////////////////////////////////////////////////////////////////////////

class LidarAltitude : public AltitudeDataProcessingBase
{
    public:

        LidarAltitude(double altitude);

        int exp_circle_size(void) override;
    
};

////////////////////////////////////////////////////////////////////////////////////////////

class PixhawkAltitude : public AltitudeDataProcessingBase
{
    public:

        PixhawkAltitude(double altitude);

        int exp_circle_size(void) override;
    
};


} // namespace altitude