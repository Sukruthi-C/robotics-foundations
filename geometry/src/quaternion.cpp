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

Quaternion Quaternion::normalize(){
    double n = std::sqrt(w*w+x*x+y*y+z*z);
    if (n<1e-7) return *this;
    return Quaternion(w/n,x/n,y/n,z/n);
}

Quaternion Quaternion::inverse(){
    double n = w*w+x*x+y*y+z*z;
    if (n<1e-7) return *this;
    Quaternion q = conjugate();
    return Quaternion(q.w/n,q.x/n,q.y/n,q.z/n);
}