#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_*x_;
  MatrixXd H_transpose = H_.transpose();
  MatrixXd S = H_*P_*H_transpose + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*H_transpose*S_inv;
  x_ = x_ + (K*y);
  MatrixXd I;
  I << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  MatrixXd Hj = CalculateJacobian(x_);
  VectorXd y = z - Hj*x_;
  MatrixXd Hj_transpose = Hj.transpose();
  MatrixXd S = Hj*P_*Hj_transpose + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*Hj_transpose*S_inv;
  x_ = x_ + (K*y);
  MatrixXd I;
  I << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;
  P_ = (I - K*Hj)*P_;
}
