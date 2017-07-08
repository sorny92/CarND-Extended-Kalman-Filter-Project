#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init() {
  
 //create a 4D state vector, we don't know yet the values of the x state
	x_ = VectorXd(4);
  Q_ = MatrixXd(4,4);

	//state covariance matrix P
	P_ = MatrixXd(4, 4);
	P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;


	//measurement covariance
	R_ = MatrixXd(2, 2);
	R_ << 0.0225, 0,
			  0, 0.0225;

	//measurement matrix
	H_ = MatrixXd(2, 4);
	H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	//the initial transition matrix F_
	F_ = MatrixXd(4, 4);
	F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
}

void KalmanFilter::Predict() {
  x_ = F_*x_;
  MatrixXd F_transpose = F_.transpose();
  P_ = F_*P_*F_transpose + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_p = H_*x_;
  VectorXd y = z - z_p;
  MatrixXd H_transpose = H_.transpose();
  MatrixXd S = H_*P_*H_transpose + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*H_transpose*S_inv;
  x_ = x_ + (K*y);
  MatrixXd I = MatrixXd(4,4);
  I << 1,0,0,0,
       0,1,0,0, 
       0,0,1,0,
       0,0,0,1;
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  MatrixXd Hj = tools.CalculateJacobian(x_);
  std::cout << Hj << endl;
  std::cout << x_ << endl;
  VectorXd z_p = Hj*x_;
  std::cout << z_p << endl;
  VectorXd y = z - z_p;
  std::cout << y << endl;
  MatrixXd Hj_transpose = Hj.transpose();
  MatrixXd S = Hj*P_*Hj_transpose + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*Hj_transpose*S_inv;
  x_ = x_ + (K*y);
  MatrixXd I = MatrixXd(4,4);
  I << 1,0,0,0,
       0,1,0,0, 
       0,0,1,0,
       0,0,0,1;
  P_ = (I - K*Hj)*P_;
}
