#include "ekf.hpp" //include ekf.hpp

void EKF::predict(const Eigen::VectorXd& u){
    state_ = f(state,u);
    Eigen::MatrixXd Fx = Jf(state,u);
    Pk_ = Fx*P*Fx.transpose()+Q;

}


void EKF::update(const Eigen::VectorXd& measure){
    // innovation residual
    Eigen::VectorXd y = measure-h(state_);

    // innnovation covariance
    Eigen::MatrixXd Hx = Jh(state_);
    Eigen::MatrixXd S = Hx*Pk_*Hx.transpose()+R;
    // compute kalman gain
    Eigen::MatrixXd kk = Pk_ * Hx.transpose()*S.inverse();

    // update the estimate
    state = state_ + kk*y;

    // update covar
    // P = (Eigen::Matrix3d::Identity() - kk*H)* Pk_; //most simple form of KF
    // update covariance — Joseph form for numerical stability
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(n, n) - kk * Hx;
    P = I_KH * Pk_ * I_KH.transpose() + kk * R * kk.transpose();

}

