project(SO3_EKF)
cmake_minimum_required(VERSION 3.12)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14 -o3")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
link_libraries(${Sophus_LIBRARIES} fmt)

# Eigen
include_directories("/usr/local/include/eigen3")
include_directories("/usr/include/eigen3")

include_directories(src)
include_directories(src/sensor_sim)

set(SRC 
src/sensor_sim/imu.cpp
src/sensor_sim/param.cpp
src/sensor_sim/utilities.cpp
)


add_executable(test_case src/test.cpp ${SRC})
