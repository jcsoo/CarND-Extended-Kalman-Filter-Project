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
  std::cout << "z=" << z << std::endl;
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // Calculate h(x) manually

  float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  float theta = atan(x_(1) / x_(0)); // check for valid range
  float rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

  VectorXd h_x = VectorXd(3);
  h_x << rho, theta, rho_dot;

  // Calculate y = z - h(x)

	VectorXd y = z - h_x;
  UpdateEKF(y);
}
