cmake_minimum_required(VERSION 3.0.0)

project(MPU6050-API)

add_library(mpu6050 src/MPU6050.cpp include/MPU6050.h)

target_link_libraries(mpu6050
    "-li2c"
)