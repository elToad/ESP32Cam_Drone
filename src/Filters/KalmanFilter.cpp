#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(){
    x.setZero();
    P.setIdentity();

    Q = Eigen::Matrix3d::Identity() * 0.001; 
    R = Eigen::Matrix3d::Identity() * 0.1;
}

void KalmanFilter::Predict(Eigen::Vector3d gyro, double dt){
    x += gyro * dt;
    P += Q;
}

void KalmanFilter::Update(Eigen::Vector3d z){
    Eigen::Matrix3d K = P * (P + R).inverse();
    x += K * (z - x);
    P = (Eigen::Matrix3d::Identity() - K) * P;
}

Eigen::Vector3d KalmanFilter::GetStates(){
    return x;
}