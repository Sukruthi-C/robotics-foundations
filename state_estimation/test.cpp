#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "kalman_filter.hpp"
int main() {
    int n = 3, m = 3;
    Eigen::Vector3d x0(0.0, 0.0, 0.0);

    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 0.01;
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.1;

    KF kf(x0, n, m, A, B, P, Q, H, R);

    std::vector<Eigen::Vector3d> measurements = {
        {1.1, 0.9, 0.05}, {2.0, 1.8, 0.10}, {3.1, 2.9, 0.15},
        {4.0, 3.8, 0.20}, {5.2, 5.1, 0.25},
    };
    std::vector<Eigen::Vector3d> controls = {
        {1.0, 1.0, 0.05}, {1.0, 1.0, 0.05}, {1.0, 1.0, 0.05},
        {1.0, 1.0, 0.05}, {1.0, 1.0, 0.05},
    };

    for (int i = 0; i < (int)controls.size(); ++i) {
        kf.predict(controls[i]);
        kf.update(measurements[i]);
        std::cout << "Step " << i+1 << " state: "
                  << kf.get_state().transpose() << "\n";
    }
    return 0;
}