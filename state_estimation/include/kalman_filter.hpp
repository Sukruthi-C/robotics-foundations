#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <iostream>
#include <vector>
#include <Eigen/Dense>


class KF{
    protected:
        int n; //state dims
        int m; //measurement dims

        Eigen::VectorXd state;
        Eigen::MatrixXd P; //Process matrix covariance
        Eigen::MatrixXd Q; //process noise covariance matrix
        Eigen::MatrixXd R; //measurement noise covariance

        Eigen::MatrixXd Pk_; //predcted covariance
        Eigen::VectorXd state_; //predicted state
        // Constructor for child classes — no A, B, H needed ->EKF/UKF
        KF(Eigen::VectorXd x0, int state_dims, int meas_dims,
        const Eigen::MatrixXd& P,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R);
    
    private:
        // only for base class -> simple KF
        Eigen::MatrixXd A; //state transition matrix
        Eigen::MatrixXd B; //control matrix
        Eigen::MatrixXd H; //measurement/observation matrix

    
    public:
        KF(Eigen::VectorXd x0, int state_dims, int meas_dims, 
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& P,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& H,
            const Eigen::MatrixXd& R);

        virtual ~KF() = default;
        virtual void predict(const Eigen::VectorXd& u);
        virtual void update(const Eigen::VectorXd& measure);
        Eigen::VectorXd get_state() const {return state;}
        Eigen::MatrixXd get_covar() const {return P;}
        
};

#endif