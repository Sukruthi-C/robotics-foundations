#include "se3.hpp"

SE3 SE3::inverse(){
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = -R_inv*t;
    return SE3(t_inv,R_inv);
}

SE3 SE3::operator*(const SE3& other) const{
    // T1*T2
    // R = R1*R2
    // t = R*t2 + t1

    return SE3(R*other.t+t,R*other.R);
}

Eigen::Vector3d SE3::operator*(const Eigen::Vector3d& point) const{
    return R*point + t;
}