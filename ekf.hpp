#include <Eigen/Dense>
#include <iostream>
#include <functional>


class EKF{


        bool DEBUG_MODE = false;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd D;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::VectorXd X_0;
        Eigen::VectorXd U_0;

        Eigen::MatrixXd P; // state covariance
        
        std::function<Eigen::MatrixXd(Eigen::MatrixXd, Eigen::MatrixXd)>f;
        Eigen::MatrixXd defaultFunc(Eigen::MatrixXd X, Eigen::MatrixXd U);


    public:
        Eigen::VectorXd X; // state_estimate
        Eigen::MatrixXd W; //Kalman Gain

        EKF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd D, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::VectorXd X_0, Eigen::VectorXd U_0, Eigen::MatrixXd P, bool debug=false);
        EKF(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd D, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::VectorXd X_0, Eigen::VectorXd U_0, Eigen::MatrixXd P, std::function<Eigen::MatrixXd(Eigen::MatrixXd, Eigen::MatrixXd)>f,  bool debug=false);

        Eigen::VectorXd computeKF(Eigen::VectorXd Z_k_1, Eigen::VectorXd U);
        void stateEstimationStep(Eigen::VectorXd Z_k_1, Eigen::VectorXd U);
        void stateCovarianceStep();







        










};