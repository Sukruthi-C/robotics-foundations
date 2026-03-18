#include "kalman_filter.hpp"

KF::KF(Eigen::VectorXd x0, int state_dims, int mea_dims, 
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& P,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& H,
            const Eigen::MatrixXd& R) :
            n(state_dims), m(mea_dims), state(x0),
            A(A),B(B),P(P),Q(Q),H(H),R(R),state_(Eigen::VectorXd::Zero(n)),Pk_(Eigen::MatrixXd::Zero(n,n)) {}

//do not redeclare virtual in cpp it should be only in hpp file
void KF::predict(const Eigen::VectorXd& u){
    /*** @brief this function is used to predict the state ahead and find the error covariance*/
    state_ = A*state + B*u;
    Pk_ = A*P*A.transpose()+Q;
}

void KF::update(const Eigen::VectorXd& measure){
    // innovation residual
    Eigen::VectorXd y = measure-H*state_;

    // innnovation covariance
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