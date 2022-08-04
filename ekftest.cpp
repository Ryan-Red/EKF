#include "ekf.hpp"
#include <iostream>
#include <cmath>



// Eigen::Matrix2d A;
// Eigen::Matrix2d B;
// Eigen::RowVector2d D;
// Eigen::Matrix2d Q;
// Eigen::MatrixXd R(1,1);
// Eigen::Vector2d X_0;
// Eigen::Vector2d U_0;

// Eigen::MatrixXd Z_0(1,1);

Eigen::MatrixXd fun(Eigen::MatrixXd X, Eigen::MatrixXd U){

    std::cout << "this is fun" << std::endl;
    return X + U;



}


int twoDimRocketExample(){

    float dt = 0.25;
    float sigma_a = 0.2;
    float epsilon = 0.1; // accelerometer measurement error
    float sigma_x_m = 20; //altimeter error


    Eigen::MatrixXd A(2,2);
    Eigen::MatrixXd B(2,1);
    Eigen::MatrixXd D(1,2);
    Eigen::MatrixXd Q(2,2);
    Eigen::MatrixXd R(1,1);
    Eigen::VectorXd X_0(2);
    Eigen::VectorXd U_0(1);

    Eigen::VectorXd Z_0(2);
    Eigen::MatrixXd P(2,2);

    A << 1, dt,
         0, 1;

    B << 0.5 * pow(dt,2), dt;

    Q << pow(dt,4)/4, pow(dt,3)/2,
         pow(dt,3)/2, pow(dt,2);

    Q = Q * pow(epsilon,2);


    D << 1, 0;

    R << pow(sigma_x_m,2);

    P << 500, 0,
           0, 500;

    X_0 << 0, 0;

    Eigen::MatrixXd Z(1,30);
    Eigen::MatrixXd a_n(1,31);
    Eigen::MatrixXd U_n(1,31);



    Z << -32.4,	-11.1,	18,	22.9,	19.5,	28.5,	46.5,	68.9,	48.2,	56.1, 90.5,	104.9,	140.9,	148,	187.6,	209.2,	244.6,	276.4,	323.5,	357.3,	357.4,	398.3,	446.7,	465.1,	529.4,	570.4,	636.8,	693.3,	707.3,	748.35;
    a_n << 2*9.81 , 39.72,	40.02,	39.97,	39.81,	39.75,	39.6,	39.77,	39.83,	39.73,	39.87,	39.81,	39.92,	39.78,	39.98,	39.76,	39.86,	39.61,	39.86,	39.74,	39.87,	39.63,	39.67,	39.96,	39.8,	39.89,	39.85,	39.9,	39.81,	39.81,	39.68;

    Eigen::MatrixXd g(1,31);
    g = -9.81 *Eigen::MatrixXd::Ones(1,31);

    U_n = a_n + g;


    EKF ekf(A, B, D, Q, R, X_0, U_0, P, true);
    Eigen::MatrixXd U_k(1,1);
    Eigen::MatrixXd Z_k(1,1);
    
    int i = 0;

    for(int i =0; i < Z.cols(); i++){
        U_k(0,0) = U_n(0,i);
        Z_k(0,0) = Z(0,i);


        std::cout <<"Computing K = " << i << std::endl;
        std::cout << ekf.computeKF(Z_k, U_k) << std::endl;
        std::cout << "----------------------"<< std::endl;

        


    }

    return 0;
}


int sixDimCarExample(){





    float dt = 1;
    float sigma_a = 0.2;

    float sigma_x = 3;
    float sigma_y = 3;

    Eigen::MatrixXd A(6,6);
    Eigen::MatrixXd B(6,6);
    Eigen::MatrixXd D(2,6);
    Eigen::MatrixXd Q(6,6);
    Eigen::MatrixXd R(2,2);
    Eigen::VectorXd X_0(6);
    Eigen::VectorXd U_0(6);

    Eigen::VectorXd Z_0(2);
    Eigen::MatrixXd P(6,6);

    A = Eigen::MatrixXd::Identity(6,6);
    A(0,1) = dt;
    A(0,2) = 0.5 * dt*dt;
    A(1,2) = dt;
    A(3,4) = dt;
    A(3,5) = 0.5 * dt*dt;
    A(4,5) = dt;

    Eigen::Matrix3d Q_A;
    Q_A << pow(dt,4)/4, pow(dt,3)/2, pow(dt,2)/2,
           pow(dt,3)/2, pow(dt,2), dt,
           pow(dt,2)/2, dt, 1;

    Q.topLeftCorner(3,3) = Q_A;
    Q.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    Q.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    Q.bottomRightCorner(3,3) = Q_A;

    Q = Q * pow(sigma_a,2);


    D = Eigen::MatrixXd::Zero(2,6);
    D(0,0) = 1;
    D(1,3) = 1;

    R << pow(sigma_x,2), 0,
         0, pow(sigma_y,2);


    B  = Eigen::MatrixXd::Zero(6,6);

    X_0 = Eigen::MatrixXd::Zero(6,1);
    U_0 = Eigen::MatrixXd::Zero(6,1);

    P = Eigen::MatrixXd::Identity(6,6) * 500;



    Eigen::MatrixXd Z(2,35);
    Z << -393.66, -375.93, -351.04,	-328.96,	-299.35,	-273.36,	-245.89,	-222.58,	-198.03,	-174.17,	-146.32,	-123.72,	-103.47,	-78.23,	-52.63,	-23.34,	25.96,	49.72,	76.94,	95.38,	119.83,	144.01,	161.84,	180.56,	201.42,	222.62,	239.4,	252.51,	266.26,	271.75,	277.4,	294.12,	301.23,	291.8,	299.89,
          300.4,	301.78,	295.1,	305.19,	301.06,	302.05,	300,	303.57,	296.33,	297.65,	297.41,	299.61,	299.6,	302.39,	295.04,	300.09,	294.72,	298.61,	294.64,	284.88,	272.82,	264.93,	251.46,	241.27,	222.98,	203.73,	184.1,	166.12,	138.71,	119.71,	100.41,	79.76,	50.62,	32.99,	2.14;



    
    EKF ekf(A, B, D, Q, R, X_0, U_0, P);
    int i = 0;

    for(auto col : Z.colwise()){

        std::cout <<"Computing K = " << i << std::endl;
        std::cout << ekf.computeKF(col, U_0) << std::endl;
        i++;
        std::cout << "----------------------"<< std::endl;




    }







    std::cout << "A:" << std::endl;
    std::cout << A <<std::endl;


    // std::cout << "Q:" << std::endl;
    // std::cout << Q << std::endl;

    // std::cout << "R:" << std::endl;
    // std::cout << R << std::endl;






    return 0;


}

int singleDim(){

    Eigen::MatrixXd A(1,1);
    Eigen::MatrixXd B(1,1);
    Eigen::MatrixXd D(1,1);
    Eigen::MatrixXd Q(1,1);
    Eigen::MatrixXd R(1,1);
    Eigen::VectorXd X_0(1,1);
    Eigen::VectorXd U_0(1,1);

    Eigen::MatrixXd Z_0(1,1);

    Eigen::MatrixXd P(1,1);

    P = Eigen::MatrixXd(1,1);
    P  << 225;

    A << 1;
    B << 0;
    D << 1;
    R << 25;
    Q << 0; 
    X_0 << 60;
    U_0 << 0;
  


    EKF ekf(A, B, D, Q, R, X_0, U_0, P);
    Z_0 << 48.54;
    std::cout <<"Computing K = 1" << std::endl;
    std::cout << ekf.computeKF(Z_0, U_0) << std::endl;

    Z_0 << 47.11;
    std::cout << "Computing K = 2" << std::endl;
    std::cout << ekf.computeKF(Z_0, U_0) << std::endl;


    Z_0 << 55.01;
    std::cout << "Computing K = 3" << std::endl;
    std::cout << ekf.computeKF(Z_0, U_0) << std::endl;

    Z_0 << 55.15;
    std::cout << "Computing K = 4" << std::endl;
    std::cout << ekf.computeKF(Z_0, U_0) << std::endl;


    Z_0 << 49.89;
    std::cout << "Computing K = 5" << std::endl;
    std::cout << ekf.computeKF(Z_0, U_0) << std::endl;



    Z_0 << 40.85;
    std::cout << "Computing K = 6" << std::endl;
    std::cout << ekf.computeKF(Z_0, U_0) << std::endl;

    return 0;

}

int main(){


    twoDimRocketExample();
    // A << 1, 1,
    //      0, 1;
    
    // B << 1, 0,
    //      0, 1;


    // D << 1, 0;

    // Q << 0, 0,
    //      0, 0;


    // R << 1;

    // X_0 << 95, 1;

    // U_0 << -4.9, -9.8;

    // Z_0 << 100;

    

    // EKF ekf(A, B, D, Q, R, X_0, U_0);



    // std::cout <<"Computing K = 1" << std::endl;
    // std::cout << ekf.compute(Z_0, U_0) << std::endl;
    // Z_0 << 97.9;
    // std::cout << "Computing K = 2" << std::endl;
    // std::cout << ekf.compute(Z_0, U_0) << std::endl;




}