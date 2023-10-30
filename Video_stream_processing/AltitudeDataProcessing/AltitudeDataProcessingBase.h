namespace altitude
{
    class BaseAltitude
    {
        public:
            virtual int exp_circle_size(void) = 0;

            virtual ~BaseAltitude() = default;
    };

}