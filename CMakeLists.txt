cmake_minimum_required(VERSION 3.18.4)

project(MPU6050-API)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread -fPIC -Wall")

add_library(mpu6050 src/MPU6050.cpp src/MPU6050.h)

target_link_libraries(mpu6050
    "-li2c"
)