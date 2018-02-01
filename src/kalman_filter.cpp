#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  std::cout << "KalmanFilter::Predict" << std::endl;
  // std::cout << "F=" << F_ << std::endl;
  // std::cout << "x=" << x_ << std::endl;
  // std::cout << "P=" << P_ << std::endl;
  // std::cout << "Q=" << Q_ << std::endl;

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  std::cout << "KalmanFilter::Update" << std::endl;
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Tools tools;
  
  std::cout << "KalmanFilter::UpdateEKF" << std::endl;
  std::cout << "x=" << x_ << std::endl;
  std::cout << "z=" << z << std::endl;
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd Hj = tools.CalculateJacobian(x_);
  std::cout << "Hj=" << Hj << std::endl;
	VectorXd z_pred = Hj * x_;
  std::cout << "z_pred=" << z_pred << std::endl;  
	VectorXd y = z - z_pred;
  std::cout << "y=" << y << std::endl;  
	MatrixXd Ht = H_.transpose();
  std::cout << "Ht=" << Ht << std::endl;  
	MatrixXd S = H_ * P_ * Ht + R_;
  std::cout << "S=" << S << std::endl;  
	MatrixXd Si = S.inverse();
  std::cout << "Si=" << Si << std::endl;  
	MatrixXd PHt = P_ * Ht;
  std::cout << "PHt=" << PHt << std::endl;  
	MatrixXd K = PHt * Si;  
  std::cout << "K=" << K << std::endl;  
	//new estimate
	x_ = x_ + (K * y);
  std::cout << "x=" << K << std::endl;  
	long x_size = x_.size();
  std::cout << "x_size=" << x_size << std::endl;  
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  std::cout << "I=" << I << std::endl;  
	P_ = (I - K * H_) * P_;  
  std::cout << "P=" << P_ << std::endl;  
}
