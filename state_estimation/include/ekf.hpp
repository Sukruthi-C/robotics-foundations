#ifndef EKF_HPP
#define EKF_HPP
#include "kalman_filter.hpp"

class EKF : public KF{
    public:
        EKF(const Eigen::VectorXd& x0, int state_dims, int meas_dims,
            const Eigen::MatrixXd& P,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R) : KF(x0,state_dims,meas_dims,P,Q,R) {}
        
        virtual ~EKF() = default; //if you dont write a destructor, c++ creates one for you but it wont be virtual
        // if you have virtual functions, your destructor needs to be virtual
        // This says: *"I want a virtual destructor, but I have nothing special to clean up 
        // — just use the compiler-generated one."* EKF itself owns no raw pointers or resources 
        // (Eigen matrices clean themselves up), so `= default` is correct here.


        virtual Eigen::VectorXd f(const Eigen::VectorXd& x, const Eigen::VectorXd& u)=0;//if yoou add =0; it becomes pure virtual and these fucntions are defined in sub classes of EKF   
        virtual Eigen::VectorXd h(const Eigen::VectorXd& x)=0;
        virtual Eigen::MatrixXd Jf(const Eigen::VectorXd& x, const Eigen::VectorXd& u)=0;
        virtual Eigen::MatrixXd Jh(const Eigen::VectorXd& x)=0;

        void predict(const Eigen::VectorXd& u) override;
        void update(const Eigen::VectorXd& measure) override;
};
#endif