#include <iostream>
#include "AltitudeDataProcessing/AltitudeDataProcessing.h"


int main()
{
    double alti = 1.0;

    altitude::PixhawkAltitude pix(alti);
    int size = pix.exp_circle_size();
    int sq = pix.exp_square_size(size);

    std::cout << size << std::endl;
    std::cout << sq << std::endl;

    return 0;
}