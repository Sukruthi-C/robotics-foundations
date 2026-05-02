#include "quaternion.hpp"

Quaternion Quaternion::operator*(const Quaternion& other) const {
    double w_new = w*other.w - x*other.x - y*other.y - z*other.z;
    double x_new = w*other.x + x*other.w + y*other.z - z*other.y;
    double y_new = w*other.y - x*other.z + y*other.w + z*other.x;
    double z_new = w*other.z + x*other.y - y*other.x + z*other.w;
    return Quaternion(w_new, x_new, y_new, z_new);
}

Quaternion Quaternion::conjugate() const{
    return Quaternion(w,-x,-y,-z);
}

// const ensures that w,x,y,z values do not change
Quaternion Quaternion::normalize() const{
    double n = std::sqrt(w*w+x*x+y*y+z*z);
    if (n<1e-7) return *this;
    return Quaternion(w/n,x/n,y/n,z/n);
}

// const ensures that w,x,y,z values do not change
Quaternion Quaternion::inverse() const{
    double n = w*w+x*x+y*y+z*z;
    if (n<1e-7) return *this;
    Quaternion q = conjugate();
    return Quaternion(q.w/n,q.x/n,q.y/n,q.z/n);
}

Eigen::Matrix3d Quaternion::toRotationMatrix() const {
    Eigen::Matrix3d m;
    Quaternion q = normalize();
    m<<1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.w*q.z),2*(q.x*q.z+q.w*q.y),
        2*(q.x*q.y+q.w*q.z),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.w*q.x),
        2*(q.x*q.z-q.w*q.y),2*(q.y*q.z+q.w*q.x),1-2*(q.x*q.x+q.y*q.y);
    return m;
}

// static in hpp only
Quaternion Quaternion::fromRotationMatrix(const Eigen::Matrix3d& R){
    double tr = R(0,0) + R(1,1) + R(2,2);
    double s=0.0,x=0.0,y=0.0,z=0.0,w=0.0;
    if (tr>0){
        s = 1/(2*std::sqrt(tr+1));
        w = 1/(4*s);
        x = (R(2,1) - R(1,2))*s;
        y = (R(0,2)-R(2,0))*s;
        z = (R(1,0) - R(0,1))*s;
    } 
    return Quaternion(w,x,y,z);
}

Eigen::Vector3d Quaternion::toAxisAngle() const{
    double theta = 2*std::acos(w);
    if (sin(theta/2)<0.0001) return Eigen::Vector3d(0,0,1);
    Eigen::Vector3d n_cap = Eigen::Vector3d(x/sin(theta/2),y/sin(theta/2),z/sin(theta/2));
    return n_cap;
}

// static belong only in hpp not cpp
Quaternion Quaternion::fromAxisAngle(const Eigen::Vector3d& axis, double angle){
    // axis should be unnit
    double w = cos(angle/2);
    Eigen::Vector3d n_cap = axis*sin(angle/2);

    return Quaternion(w,n_cap(0),n_cap(1),n_cap(2));
}

Quaternion Quaternion::slerp(const Quaternion& other, double t) const {
    double d = w*other.w + x*other.x + y*other.y + z*other.z;

    // if d < 0, short path: flip other and negate d
    Quaternion q2 = other;
    if (d < 0) {
        q2 = Quaternion(-other.w, -other.x, -other.y, -other.z);
        d = -d;
    }

    // if quaternions are very close, fallback to lerp + normalize
    if (d > 0.9995) {
        Quaternion result(w + t*(q2.w - w),
                          x + t*(q2.x - x),
                          y + t*(q2.y - y),
                          z + t*(q2.z - z));
        return result.normalize();
    }

    // slerp formula — fill in the blanks:
    double theta = std::acos(d);
    double sin_theta = std::sin(theta);

    double s1 = std::sin((1-t)*theta) / sin_theta;
    double s2 = std::sin(t*theta) / sin_theta;
    
    return Quaternion(
        s1*w   + s2*q2.w,
        s1*x   + s2*q2.x,
        s1*y   + s2*q2.y,
        s1*z   + s2*q2.z
    );
}

double Quaternion::angularDistance(const Quaternion& other) const {
    double d = w*other.w + x*other.x + y*other.y + z*other.z;
    return 2.0 * std::acos(std::clamp(std::abs(d), -1.0, 1.0));
}