#include <Eigen/Dense>
#include <iostream>


class EKF{

        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd D;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::VectorXd X_0;
        Eigen::VectorXd U_0;

        Eigen::MatrixXd P; // state covariance


    public:
        Eigen::VectorXd X; // state_estimate
        Eigen::MatrixXd W; //Kalman Gain




        void stateCovarianceStep(){ //Estimate the state covariance
            std::cout << ":)" << std::endl;
            Eigen::MatrixXd P_k_1_k = A * P * A.transpose() + Q;

            // std::cout << "P_k_1_k is of: " << std::endl;
            // std::cout << P_k_1_k << std::endl;


            Eigen::MatrixXd S = D * P_k_1_k * D.transpose() + R;

            //std::cout << "S is: " << std::endl;
            //std::cout << S <<std::endl;
            W = P_k_1_k * D.transpose() * S.inverse();
            P = P_k_1_k - W * S * W.transpose();

            // std::cout << "W is" << std::endl;
            // std::cout << W << std::endl;

            // std::cout << "P is" << std::endl;
            // std::cout << P << std::endl;




        }

        void stateEstimationStep(Eigen::VectorXd Z_k_1, Eigen::VectorXd U){
            Eigen::VectorXd X_k_1_k = A * X + B * U;
            std::cout << "x_k_1_k" << std::endl;
            std::cout << A*X << std::endl;
            Eigen::VectorXd Z_k_1_k = D * X_k_1_k;


            //std::cout << "Z_k_1_k" << std::endl;
            //std::cout << Z_k_1_k << std::endl;


            // std::cout << "hi" << std::endl;



            // std::cout <<Z_k_1.rows() << ", " <<  Z_k_1.cols() << std::endl;
            //  std::cout <<Z_k_1_k.rows() << ", " <<  Z_k_1_k.cols() << std::endl;
            Eigen::VectorXd s_k_1 = Z_k_1 - Z_k_1_k;
            // std::cout << "\ns_k_1" << std::endl;
            // std::cout << s_k_1<< std::endl;
            
            X = X_k_1_k + W * s_k_1;
             //std::cout << "\nX_k_1_k" << std::endl;
             //std::cout << X_k_1_k <<std::endl;
            // std::cout << "\n\n" << std::endl;
        }

        Eigen::VectorXd compute(Eigen::VectorXd Z_k_1, Eigen::VectorXd U){
            stateCovarianceStep();
            // std::cout << "Finished Covariance Step\n";
            stateEstimationStep(Z_k_1, U);

            return X;
        }









    public:
        EKF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd D, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::VectorXd X_0, Eigen::VectorXd U_0, Eigen::MatrixXd P);





        










};