#include "IAltitudeDataProcessing.h"


namespace altitude
{

class AltitudeDataProcessingBase : public IAltitudeDataProcessing
{
    public:

        AltitudeDataProcessingBase(double altitude);

        int exp_square_size(int circle_size) override;

    protected:

        double m_altitude;

};


} // namespace altitude