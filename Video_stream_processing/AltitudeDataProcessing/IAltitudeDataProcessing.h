namespace altitude
{

class IAltitudeDataProcessing
{
    public:

        virtual int exp_circle_size(void) = 0;

        virtual int exp_square_size(int circle_size) = 0;

        virtual ~IAltitudeDataProcessing() = default;

};

} // namespace altitude