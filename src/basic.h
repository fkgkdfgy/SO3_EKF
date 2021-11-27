#pragma once
#include <iostream>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Eigen"
#include "sophus/so3.hpp"
#include "glog/logging.h"


using V3d = Eigen::Vector3d;
using M3d = Eigen::Matrix3d;

struct IMUData
{
    double timestamp;
    V3d Acc;
    V3d Gyr;
};

struct GPSData
{
    double timestamp;
    V3d position;
    V3d orientation;

    Eigen::Matrix3d position_cov;
    Eigen::Matrix3d orientation_cov;
    Eigen::Matrix3d postion_info;
    Eigen::Matrix3d orientation_info;
};

Eigen::Matrix3d RightJacobian(Eigen::Vector3d input)
{
    Eigen::Matrix3d result;
    const double eps = 1e-4;
    const double d = input.norm();
    const double d2 = d*d;

    Eigen::Matrix3d W = Sophus::SO3d::hat(input);
    Eigen::Matrix3d I;
    I.setIdentity();
    if(d<eps)
    {
        return I;
    }
    else
    {
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Eigen::Matrix3d InverseRightJacobian(Eigen::Vector3d input)
{
    return RightJacobian(input).inverse();
}