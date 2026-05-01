#pragma once
#include <cmath>
#include <Eigen/Dense>

class Quaternion{
    public:
        double w,x,y,z;

        // constructors
        Quaternion(double w, double x, double y, double z)

        // core
        Quaternion operator*(const Quaternion& other) const;
        Quaternion conjugate() const;
        Quaternion inverse() const;
        Quaternion normalize();

        // conversion
        Eigen::Matrix3d toRotationMatrix() const;
        static Quaternion fromRotationMatrix(const Eigen::Matrix3d& R);
        Eigen::Vector3d toAxisAngle() const;
        static Quaternion fromAxisAngle(const Eigen::Vector3d& axis, double angle);

        // interpolation/metric
        Quaternion slerp(const Quaternion& other,double t) const;
        double angularDistance(const Quaternion& other) const; 
};