
#include "ekf.hpp"


Eigen::MatrixXd EKF::defaultFunc(Eigen::MatrixXd X, Eigen::MatrixXd U){


        return A * X + B * U;
}


EKF::EKF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd D, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::VectorXd X_0, Eigen::VectorXd U_0, Eigen::MatrixXd P, bool debug):
        A(A),
        B(B),
        D(D),
        Q(Q),
        R(R),
        X(X_0),
        U_0(U_0),
        P(P),
        f([this](Eigen::MatrixXd X, Eigen::MatrixXd U)-> Eigen::MatrixXd {
                return this->A * X + this->B * U;
                }),
        DEBUG_MODE(debug)
        {
        }


EKF::EKF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd D, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::VectorXd X_0, Eigen::VectorXd U_0, Eigen::MatrixXd P, std::function<Eigen::MatrixXd(Eigen::MatrixXd, Eigen::MatrixXd)>f, bool debug):
        A(A),
        B(B),
        D(D),
        Q(Q),
        R(R),
        X(X_0),
        U_0(U_0),
        P(P),
        f(f),
        DEBUG_MODE(debug)
        {}


Eigen::VectorXd EKF::computeKF(Eigen::VectorXd Z_k_1, Eigen::VectorXd U){
            stateCovarianceStep();
            stateEstimationStep(Z_k_1, U);

            return X;
}


void EKF::stateEstimationStep(Eigen::VectorXd Z_k_1, Eigen::VectorXd U){ //Perform the state estimation (needs to preceed the state estimation to get the Kalman Gain)
            
            
            Eigen::VectorXd X_k_1_k = f(X,U);
            Eigen::VectorXd Z_k_1_k = D * X_k_1_k;
            Eigen::VectorXd s_k_1 = Z_k_1 - Z_k_1_k;

            
            X = X_k_1_k + W * s_k_1;


            if(DEBUG_MODE){

                std::cout << "\nZ_k_1_k" << std::endl;
                std::cout << Z_k_1_k << std::endl;

                std::cout << "\nX_k_1_k" << std::endl;
                std::cout << X_k_1_k <<std::endl;
                std::cout << "\n\n" << std::endl;

                std::cout << "\ns_k_1" << std::endl;
                std::cout << s_k_1<< std::endl;
            }

}
void EKF::stateCovarianceStep(){ //Estimate the state covariance

            Eigen::MatrixXd P_k_1_k = A * P * A.transpose() + Q;
            Eigen::MatrixXd S = D * P_k_1_k * D.transpose() + R;

            W = P_k_1_k * D.transpose() * S.inverse();
            P = P_k_1_k - W * S * W.transpose();



            if(DEBUG_MODE){

                std::cout << "\nP_k_1_k is of: " << std::endl;
                std::cout << P_k_1_k << std::endl;

                std::cout << "\nS is: " << std::endl;
                std::cout << S <<std::endl;

                std::cout << "\nW is:" << std::endl;
                std::cout << W << std::endl;

                std::cout << "\nP is: " << std::endl;
                std::cout << P << std::endl;

            }



}
