#include "geometry/se2.hpp"

// you are negating the rotation matrix here
// if you are currently going from A -> B
// inverse takes you from B -> A
SE2 SE2::inverse() const{

    // manual calculation
    // double inv_theta = -theta;
    // double inv_x = -std::cos(theta)*x - std::sin(theta)*y;
    // double inv_y = std::sin(theta)*x - std::cos(theta)*y;
    // return SE2(inv_x,inv_y,inv_theta);

    // use Eigen
    Eigen::Matrix2d R;
    R<< cos(theta),-sin(theta),
        sin(theta),cos(theta);
    
    Eigen::Vector2d t(x,y);

    Eigen::Matrix2d R_inv = R.transpose();
    Eigen::Vector2d t_inv = -R_inv*t;
    
    return SE2(t_inv.x(),t_inv.y(),theta);
}