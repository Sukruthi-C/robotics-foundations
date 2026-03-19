#ifndef SE3_HPP
#define SE3_HPP

#include <Eigen/Dense>
#include <cmath>

class SE3{

    public:
        Eigen::Vector3d t;
        Eigen::Matrix3d R;
        
        /** @brief constructor*/
        SE3(const Eigen::Vector3d t_, const Eigen::Matrix3d& R_) : t(t_),R(R_) {}
        
        /** @brief function calculates and returns the inverse of the matrix*/
        SE3 inverse() const;


        /** @brief function multiplies two transforms*/ 
        SE3 operator*(const SE3& other) const;

        /** @brief apply trasnform to a 3d point */

        Eigen::Vector3d operator*(const Eigen::Vector3d& point) const;
};

#endif