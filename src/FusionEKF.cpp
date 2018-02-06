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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
	H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;
  
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

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

	// noise_ax = 5;
	// noise_ay = 5;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  // if (measurement_pack.sensor_type_ != MeasurementPackage::RADAR) {
  //   return;
  // }
  cout << measurement_pack.timestamp_ << endl;
  if (abs(measurement_pack.timestamp_ - previous_timestamp_) > 100000) {
    // cout << "Timestamp out of range - restarting" << endl;
    is_initialized_ = false;
  }

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    const VectorXd& raw = measurement_pack.raw_measurements_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout << " z: " << raw(0) << " " << raw(1) << " " << raw(2) << std::endl;
      ekf_.x_ = ToCartesian(raw);      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << raw(0), raw(1), 0, 0;
    }
  	previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    std::cout << " x: " << ekf_.x_(0) << " " << ekf_.x_(1) << " " << ekf_.x_(2) << " " << ekf_.x_(3) << std::endl;
    VectorXd p = ToPolar(ekf_.x_);
    std::cout << " r: " << p(0) << " " << p(1) << " " << p(2) << endl;
    std::cout << endl;
    return;
  }


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

	//compute the time elapsed between the current and previous measurements

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

	float noise_ax = 9;
	float noise_ay = 9;


	//1. Modify the F matrix so that the time is integrated

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  

	//2. Set the process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);

  ekf_.Q_ << 
      dt_4 * noise_ax / 4.0, 0, dt_3 * noise_ax / 2.0, 0,
      0, dt_4 * noise_ay / 4.0, 0, dt_3 * noise_ay / 2.0,
      dt_3 * noise_ax / 2.0, 0, dt_2 * noise_ax, 0,
      0, dt_3 * noise_ay / 2.0, 0, dt_2 * noise_ay;



  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();
  // std::cout << " x: " << ekf_.x_(0) << " " << ekf_.x_(1) << " " << ekf_.x_(2) << " " << ekf_.x_(3) << std::endl;

  // cout << "Predict Done" << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  const VectorXd& z = measurement_pack.raw_measurements_;
  const VectorXd& x = ekf_.x_;
  MatrixXd H;
  MatrixXd Hx;
  MatrixXd R;


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    H = CalculateJacobian(ToCartesian(z));
    Hx = ToPolar(x);
    R = R_radar_;
  } else {
    H = H_laser_;
    Hx = H * x;
    R = R_laser_;
  }
  ekf_.Update(z, H, Hx, R);
  std::cout << " x: " << ekf_.x_(0) << " " << ekf_.x_(1) << " " << ekf_.x_(2) << " " << ekf_.x_(3) << std::endl;
  std::cout << " P:\n" << ekf_.P_ << endl;
  std::cout << endl;
  // // print the output
  
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
