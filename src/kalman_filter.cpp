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
  // std::cout << "x: " << x_ << std::endl;
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R) {
  std::cout << "KalmanFilter::Update" << std::endl;  
  // std::cout << "x: " << x_ << std::endl;
  // std::cout << "p: " << P_ << std::endl;
  // std::cout << "z: " << z << std::endl;
  // std::cout << "H: " << H << std::endl;
  // std::cout << "Hx: " << Hx << std::endl;
  // std::cout << "R: " << R << std::endl;


	const MatrixXd PHt = P_ * H.transpose();
	const MatrixXd S = H * PHt + R;
	const MatrixXd K = PHt * S.inverse();
	VectorXd y = z - Hx;
  if (y.size() == 3) { 
    y(1) = atan2(sin(y(1)), cos(y(1)));    
  }
  std::cout << " z: " << z(0) << " " << z(1) << " " << z(2) << std::endl;  
  std::cout << "Hx: " << Hx(0) << " " << Hx(1) << " " << Hx(2) << std::endl;  
  std::cout << " y: " << y(0) << " " << y(1) << " " << y(2) << std::endl;  

  std::cout << " K:\n" << K << std::endl;  

  VectorXd ky = K * y;
  std::cout << "Ky: " << ky(0) << " " << ky(1) << " " << ky(2) << " " << ky(3) << std::endl;  

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * H) * P_;
  // std::cout << " P: " << P_ << std::endl;  
}

