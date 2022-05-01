//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

#pragma once
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
#include <cmath>
#include <thread>

#define _POSIX_C_SOURCE 200809L //Used for calculating time

#define TAU 0.05 //Complementary filter percentage
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)

int CONFIG[4] = {0b00000000, 0b00001000, 0b00010000, 0b00011000};
float ACCEL_SENS[4] = {16384.0, 8192.0, 4096.0, 2048.0};
float GYRO_SENS[4] = {131.0, 65.5, 32.8, 16.4};

struct CalibrationData {
	uint8_t gyro_range = -1;
	float gyro_offset_X = 0, gyro_offset_Y = 0, gyro_offset_Z = 0;
	uint8_t accel_range = -1;
	float accel_offset_X = 0, accel_offset_Y = 0, accel_offset_Z = 0;
};

class MPU6050 {
	private:
		void _update_loop();
		float _accel_angle[3];
		float _gyro_angle[3];
		float _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

		float ax, ay, az, gr, gp, gy; //Temporary storage variables used in _update()

		int MPU6050_addr;
		int f_dev; //Device file

		float dt; //Loop time (recalculated with each loop)

		struct timespec start,end; //Create a time structure

		bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter
		CalibrationData _calibration;
	public:
		MPU6050(int8_t addr, CalibrationData calibration_data);
		MPU6050(int8_t addr, CalibrationData calibration_data, bool run_update_thread);
		void getAccelRaw(volatile float *x, volatile float *y, volatile float *z);
		void getGyroRaw(float *roll, float *pitch, float *yaw);
		bool getAccel(volatile float *x, volatile float *y, volatile float *z);
		void getGyro(float *roll, float *pitch, float *yaw);
		void getOffsets(const uint16_t samples, float *ax_off, float *ay_off, float *az_off, float *gr_off, float *gp_off, float *gy_off);
		int getAngle(int axis, float *result);
		bool calc_yaw;
        int clearInterrupt();
		void manual_update();
};
