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

void KalmanFilter::Predict(float dt) {
  // std::cout << "KalmanFilter::Predict" << std::endl;
  // std::cout << "x: " << x_ << std::endl;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
	// 1. Modify the F matrix so reflect the relative time step

  F_(0, 2) = dt;
  F_(1, 3) = dt;

	//2. Set the process covariance matrix Q

  float q11 = dt_4 * noise_ax_ / 4.0;
  float q13 = dt_3 * noise_ax_ / 2.0;
  float q22 = dt_4 * noise_ay_ / 4.0;
  float q24 = dt_3 * noise_ay_ / 2.0;
  float q31 = q13;
  float q33 = dt_2 * noise_ax_;
  float q42 = q22;
  float q44 = dt_2 * noise_ay_;

	Q_ = MatrixXd(4, 4);
  Q_ <<
      q11,   0, q13,   0,
        0, q22,   0, q24,
      q31,   0, q33,   0,
        0, q42,   0, q44;


	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R) {
  // std::cout << "KalmanFilter::Update" << std::endl;  

	const MatrixXd PHt = P_ * H.transpose();
	const MatrixXd S = H * PHt + R;
	const MatrixXd K = PHt * S.inverse();
	VectorXd y = z - Hx;
  if (y.size() == 3) { 
    y(1) = atan2(sin(y(1)), cos(y(1)));    
  }

  // std::cout << " z: " << z(0) << " " << z(1) << " " << z(2) << std::endl;  
  // std::cout << "Hx: " << Hx(0) << " " << Hx(1) << " " << Hx(2) << std::endl;  
  // std::cout << " y: " << y(0) << " " << y(1) << " " << y(2) << std::endl;  
  // std::cout << " K:\n" << K << std::endl;  

  // VectorXd ky = K * y;
  // std::cout << "Ky: " << ky(0) << " " << ky(1) << " " << ky(2) << " " << ky(3) << std::endl;  

	x_ = x_ + (K * y);
	P_ = (I_ - K * H) * P_;
}

