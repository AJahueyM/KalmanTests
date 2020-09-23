cmake_minimum_required(VERSION 3.17)
project(Kalman)

set(CMAKE_CXX_STANDARD 20)
include_directories(/usr/include/python3.8 include)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
link_libraries(python3.8)
add_executable(Kalman src/main.cpp src/SimpleKalmanFilter/SimpleKalman.cpp)

