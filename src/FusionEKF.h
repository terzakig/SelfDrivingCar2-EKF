#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"




class FusionEKF : public KalmanFilter{
public:
  const double inf_variance = 100000;
  /**
  * Constructor.
  */
  FusionEKF();
  // inherited from interface KalmanFilter
  void Init(const MeasurementPackage&);
  // inherited from interface KalmanFilter
  void Predict();
  // inherited from interface KalmanFilter 
  void Update(const MeasurementPackage&);
  
  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage&);
  
 

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Eigen::MatrixXd Cov_laser_;
  Eigen::MatrixXd Cov_radar_;
  //Eigen::MatrixXd H_laser_;
  //Eigen::MatrixXd Hj_;
  // Make some (acceleration) noise!
  double noise_ax, noise_ay;
};

#endif /* FusionEKF_H_ */
