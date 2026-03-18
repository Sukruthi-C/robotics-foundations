#include "ekf.hpp" //include ekf.hpp

void EKF::predict(const Eigen::VectorXd& u){
    // TODO:
    state_ = f(state,u);
    Fx = Jf(state,u)
    Pk_ = Fx*P*Fx.transpose()+Q;

}


void EKF::update(const Eigen::VectorXd& measure){
    // TODO:
    // innovation residual
    Eigen::VectorXd y = measure-h(state_);

    // innnovation covariance
    H = Jh(state_);
    Eigen::MatrixXd S = H*Pk_*H.transpose()+R;
    // compute kalman gain
    Eigen::MatrixXd kk = Pk_ * H.transpose()*S.inverse();

    // update the estimate
    state = state_ + kk*y;

    // update covar
    // P = (Eigen::Matrix3d::Identity() - kk*H)* Pk_; //most simple form of KF
    // update covariance — Joseph form for numerical stability
    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(n, n) - kk * H;
    P = I_KH * Pk_ * I_KH.transpose() + kk * R * kk.transpose();

}

