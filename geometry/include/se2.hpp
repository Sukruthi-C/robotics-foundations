#ifndef SE2_HPP
#define SE2_HPP

#include <Eigen/Dense>
#include <cmath>

class SE2{

    public:
        double x;
        double y;
        double theta;
        
        /** @brief constructor*/
        SE2(const double& x_,const double& y_,const double& theta_) : x(x_),y(y_),theta(theta_) {}
        
        /** @brief function calculates and returns the inverse of the matrix*/
        SE2 inverse() const;


        /** @brief function multiplies two matrix*/ 
        SE2 operator*(const SE2& other) const;
};

#endif