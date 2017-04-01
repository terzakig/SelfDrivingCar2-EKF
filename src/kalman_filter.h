#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include <vector>
#include <iostream>

#include "measurement_package.h"


class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd Sigma_;

  // state transistion matrix (provided some sort of linear transition)
  Eigen::MatrixXd F_;

  // List of measurement covariance matrices (for multiple sensors)
  std::vector<Eigen::MatrixXd> vQ_;
  
  // list of measurement matrix/Jacobians
  std::vector<Eigen::MatrixXd> vH_;

  // Transition (motion model) covariance matrix
  Eigen::MatrixXd R_;

  /**
   * Constructor
   */
  KalmanFilter() {};

  /**
   * Destructor
   */
  virtual ~KalmanFilter() {};

  
  // The following should essentially allocate most of the matrices and initialize the prior distribution
  virtual void Init(const MeasurementPackage&) = 0;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  virtual void Predict() = 0;

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  virtual void Update(const MeasurementPackage&) = 0;

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  //void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
