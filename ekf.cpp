
#include "ekf.hpp"


EKF::EKF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd D, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::VectorXd X_0, Eigen::VectorXd U_0, Eigen::MatrixXd P):
        A(A),
        B(B),
        D(D),
        Q(Q),
        R(R),
        X(X_0),
        U_0(U_0),
        P(P) {
        
}



