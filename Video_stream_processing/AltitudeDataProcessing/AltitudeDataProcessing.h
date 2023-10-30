#include <cmath>
#include "AltitudeDataProcessingBase.h"

namespace altitude
{
    class DefaultAltitude : public BaseAltitude
    {
        public:

            DefaultAltitude(double altitude) : altitude_(altitude) {};

            int exp_circle_size(void) override;
        
        private:

            double altitude_;
    };

    class LidarAltitude : public BaseAltitude
    {
        public:

            LidarAltitude(double altitude) 
            {
                altitude_ = altitude * 100 - 10;
            };

            int exp_circle_size(void) override;
        
        private:

            double altitude_;
    };

    class PixhawkAltitude : public BaseAltitude
    {
        public:

            PixhawkAltitude(double altitude) 
            {
                altitude_ = altitude * 100;
            };

            int exp_circle_size(void) override;
        
        private:

            double altitude_;
    };


}