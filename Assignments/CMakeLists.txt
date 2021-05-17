cmake_minimum_required(VERSION 2.8)
project(KalmanFilterwithlidar)

add_definitions(-std=c++11)

# set EIGEN_DIR variable to Eigen Library Path
SET(EIGEN_INCL_DIR "${${PROJECT_NAME}_SOURCE_DIR}")

# include the directory
include_directories(${EIGEN_DIR})

# add_executable ("${PROJECT_NAME}" src1/KF.cpp)
# add_executable ("${PROJECT_NAME}" src2/main.cpp src2/kalman_filter.cpp src2/tracking.cpp)
# add_executable ("${PROJECT_NAME}" src3/jacobian.cpp)
# add_executable ("${PROJECT_NAME}" src3/rmse.cpp)
 add_executable ("${PROJECT_NAME}" src4/main.cpp src4/ukf.cpp)
