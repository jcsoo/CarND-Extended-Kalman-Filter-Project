#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // identity matrix
  Eigen::MatrixXd I_;

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;

  Eigen::MatrixXd H_laser_;

  float noise_ax_;
  float noise_ay_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(float dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::VectorXd &Hx, const Eigen::MatrixXd &R);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateRadar(const Eigen::Vector3d &z);
  void UpdateLaser(const Eigen::Vector2d &z);
};

#endif /* KALMAN_FILTER_H_ */
