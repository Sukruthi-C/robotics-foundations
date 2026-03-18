#include "geometry/se2.hpp"

SE2 SE2::inverse() const{
    double inv_theta = -theta;
    double inv_x = -std::cos(theta)*x - std::sin(theta)*y;
    double inv_y = std::sin(theta)*x - std::cos(theta)*y;
    return SE2(inv_x,inv_y,inv_theta);
}