#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &Q_in) {                         
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  std::cout << "KalmanFilter::Predict" << std::endl;
  std::cout << "x: " << x_ << std::endl;
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R) {
  std::cout << "KalmanFilter::Update" << std::endl;  
  std::cout << "x: " << x_ << std::endl;
  std::cout << "p: " << P_ << std::endl;
  std::cout << "z: " << z << std::endl;
  std::cout << "H: " << H << std::endl;
  std::cout << "Hx: " << Hx << std::endl;
  std::cout << "R: " << R << std::endl;


	MatrixXd PHt = P_ * H.transpose();
  std::cout << "PHt: " << PHt << std::endl;
	MatrixXd S = H * PHt + R;
  std::cout << "S: " << S << std::endl;
	MatrixXd K = PHt * S.inverse();
  std::cout << "K: " << K << std::endl;

	VectorXd y = z - Hx;
  std::cout << "y: " << y << std::endl;
  if (y.size() == 3) { 
    std::cout << "normalize y" << std::endl;
    y(1) = atan2(sin(y(1)), cos(y(1)));    
    std::cout << "y: " << y << std::endl;
  }

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * H) * P_;
  std::cout << " x: " << x_(0) << " " << x_(1) << " " << x_(2) << " " << x_(3) << std::endl;
  std::cout << " P: " << P_ << std::endl;
  
}

