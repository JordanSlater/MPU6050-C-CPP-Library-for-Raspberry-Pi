#include <iostream>
#include <iomanip>

#include "mpu6050/mpu6050.h"

MPU6050 mpu_device(0x68, false);

uint16_t getSampleSize(int argc, char *argv[]) {
    uint16_t sample_size = 1000;
    if (argc == 2)
        sample_size = std::stoi(argv[1]);
    else
        std::cout << "You can specify the number of samples to take while calibrating." << std::endl
            << "\tUsing the default number of samples: " << sample_size << "." << std::endl;
    return sample_size;
}

int main(int argc, char *argv[]) {
    uint16_t sample_size = getSampleSize(argc, argv);
    float ax, ay, az, gr, gp, gy;
	std::cout << "Calculating the offsets..." << std::endl
        << "\tPlease keep the accelerometer level and still" << std::endl
        << "\tThis could take a couple of minutes..." << std::endl;
	mpu_device.getOffsets(sample_size, &ax, &ay, &az, &gr, &gp, &gy);
    const int w = 10;
	std::cout << "Gyroscope R,P,Y:     " << std::setw(w) << gr << "," << std::setw(w) << gp << "," << std::setw(w) << gy << std::endl
        << "Accelerometer X,Y,Z: " << std::setw(w) << ax << "," << std::setw(w) << ay << "," << std::setw(w) << az << "\n";
}
