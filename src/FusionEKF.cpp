#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	//state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);


	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;


  ekf_.I_ = MatrixXd::Identity(4, 4);

  //measurement covariance matrix - laser
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_laser_ << 0.0225, 0,
        0, 0.0225;

  // // //measurement covariance matrix - radar
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.H_laser_ <<
    1, 0, 0, 0,
	  0, 1, 0, 0;

  ekf_.noise_ax_ = 9;
  ekf_.noise_ay_ = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  auto timestamp = measurement_pack.timestamp_;
  auto sensor_type = measurement_pack.sensor_type_;
  auto z = measurement_pack.raw_measurements_;

  // if (measurement_pack.sensor_type_ != MeasurementPackage::RADAR) {
  //   return;
  // }

  // Check for timestamp discontinuities and restart if detected.

  if (abs(measurement_pack.timestamp_ - previous_timestamp_) > 100000) {
    is_initialized_ = false;
  }

  // Initialize the EKF if this is the first measurement or if the timestamp has
  // jumped.

  if (!is_initialized_) {

    // Initialize the state covariance matrix P

    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    if (sensor_type == MeasurementPackage::RADAR) {
      ekf_.x_ = ToCartesian(z);      
    } else if (sensor_type == MeasurementPackage::LASER) {
      ekf_.x_ << z(0), z(1), 0, 0;
    }

    is_initialized_ = true;
  	previous_timestamp_ = timestamp;    
    return;

  }

  ekf_.Predict((timestamp - previous_timestamp_) / 1000000.0);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (sensor_type == MeasurementPackage::RADAR) {
    ekf_.UpdateRadar(z);
  } else if (sensor_type == MeasurementPackage::LASER) {
    ekf_.UpdateLaser(z);
  }

  // const VectorXd& z = measurement_pack.raw_measurements_;
  // const VectorXd& x = ekf_.x_;
  // MatrixXd H;
  // MatrixXd Hx;
  // MatrixXd R;


  // if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  //   H = CalculateJacobian(ToCartesian(z));
  //   Hx = ToPolar(x);
  //   R = R_radar_;
  // } else {
  //   H = H_laser_;
  //   Hx = H * x;
  //   R = R_laser_;
  // }
  // ekf_.Update(z, H, Hx, R);

  // std::cout << " x: " << ekf_.x_(0) << " " << ekf_.x_(1) << " " << ekf_.x_(2) << " " << ekf_.x_(3) << std::endl;
  // std::cout << " P:\n" << ekf_.P_ << endl;
  // std::cout << endl;
  // // // print the output
  
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;

  previous_timestamp_ = timestamp;
}
