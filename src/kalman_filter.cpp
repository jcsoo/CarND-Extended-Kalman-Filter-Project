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

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  Tools tools;
  
  std::cout << "KalmanFilter::UpdateEKF" << std::endl;
  std::cout << "z=" << z << std::endl;
  
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // Calculate h(x) manually

  float rho = sqrt(px * px + py * py);
  float theta = atan2(py, px); 
  float rho_dot = (px * vx + py * vy) / rho;

  while (theta > M_PI) {
    theta -= 2 * M_PI;
  }

  while (theta < -M_PI) {
    theta += 2 * M_PI;
  }

  std::cout << "rho: " << rho << " theta: " << theta << " rho_dot " << rho_dot << std::endl;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  cout << "z_pred=" << z_pred << std::endl;
	VectorXd y = z - z_pred;
  cout << "y=" << y << std::endl;
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

void KalmanFilter::Update(const VectorXd &z) {
  std::cout << "KalmanFilter::Update" << std::endl;  
  std::cout << z << std::endl;
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

