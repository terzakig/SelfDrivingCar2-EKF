#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using namespace Eigen;
using std::vector;



/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // allocating matrices
  Cov_laser_ = MatrixXd(2, 2);
  Cov_radar_ = MatrixXd(3, 3);
  

  //measurement covariance matrix - laser
  Cov_laser_ << 0.0225, 0, // 0.0225
              0, 0.0225;
  /*Cov_laser_<< 5, 0, // 0.0225
               0, 5;*/
  //measurement covariance matrix - radar
  Cov_radar_ << 0.09, 0, 0,
              0, 0.0009, 0, // rasinig from the original 0.0009
              0, 0, 0.0009; // hiking it up from 0.09 (original)
  /*Cov_radar_ << 5, 0, 0,
              0, 5, 0, 
              0, 0, 5;  */
  
  noise_ax = noise_ay = 5; 
  
  
}
// Initialization of the prior 
// either from a Lidar or a Radar measurement.
void FusionEKF::Init(const MeasurementPackage &pack) {
	// preliminary allocation
	x_ = VectorXd(4);
	
	if (pack.sensor_type_ == MeasurementPackage::LASER) { // direct initialization from the Lidar data
	  
	  //cout<<"Initial LIDAR measurement : "<<endl<<pack.raw_measurements_<<endl;
	  double px = pack.raw_measurements_[0], 
		 py = pack.raw_measurements_[1];
	  if (px*px + py*py == 0) { // initialize prior with high uncertainty
	    x_ << 0,
		  0,
		  0,
		  0;
	    Sigma_ = FusionEKF::inf_variance * MatrixXd::Identity(4, 4);
	  } 
	  else {
	    // state mean
	    x_<< px, 
		 py,
		  0, 
	          0;
	
	  // state covariance
	  Sigma_ = MatrixXd(4, 4);
	  Sigma_ << Cov_laser_(0, 0), Cov_laser_(0, 1), 0, 0,
		    Cov_laser_(1, 0), Cov_laser_(1, 1), 0, 0,
			    0        ,         0       , FusionEKF::inf_variance, 0,
			    0        ,         0       , 0, FusionEKF::inf_variance;  // relatively large for "dont really know, but maaaaybe zero..."
	  }
	  
	} else { // Oh-oh... Radar measurement...
	 
	  // obtain px, py implicitly
	  double rho = pack.raw_measurements_[0];
	  //cout<<"Initial RADAR measurement : "<<endl<<pack.raw_measurements_<<endl;
	  if (rho == 0) { // initialize the prior with high uncertainty around zero...
	    x_ <<  0, 
		   0,
		   0,
		   0;
	    Sigma_ = FusionEKF::inf_variance * MatrixXd::Identity(4, 4);
	  } 
	  else {
	    double phi = pack.raw_measurements_[1];
	    double px = rho * cos(phi);
	    double py = rho * sin(phi);
	  
	    // *** Ok, about the velocity now... ****
	    // get the rho_dot:
	    double rho_dot = pack.raw_measurements_[2];
	    // We know that rho_dot is the dot product of v and normalized [cos(θ); sin(θ)]. 
	    // Thus, norm(v) = abs(rho_dot):
	    // Now, we know that the component of v along rho is rho_dot adn there is no way to recover
	    // the component of v that is perpendicular to rho. Thus, we assume it is a 0-mean very uncertain Gaussian...
	    // So, in the mean we simply use rho_dot * [cos(θ); sin(θ)]
	    double vx = rho_dot * cos(phi), 
		   vy = rho_dot * sin(phi); 
	  
	    // assigning the state mean
	    x_ << px,
		  py,
		  vx,
		  vy;
	  
	    // Now we need a variance. Propagating uncertainty using the linear approximation of 
	    // the state vector wrt radar variables:
	    //
	    // x ~= [cos(phi),        -rho*sin(phi),            0; 
	    //	   sin(phi),         rho*cos(phi),            0;
	    //	      0    ,        -rho_dot * sin(phi),   cos(phi);
	    //	      0    ,         rho_dot * cos(phi),   sin(phi)] * ( [rho; phi; rho_dot] - z_r ) 
	    // So here's the Jacobian:
	    MatrixXd J(4, 3);
	    J <<     cos(phi),        -rho * sin(phi),            0,
		     sin(phi),         rho * cos(phi),            0,
	               0     ,      -rho_dot * sin(phi),    cos(phi),
	               0     ,       rho_dot * cos(phi),    sin(phi);
	    // Great! Now finding the approximate covariance of the prior:
	    Sigma_ = J * Cov_radar_ * J.transpose();
	  
	    // Now adding extra variance to the velocity components
	    Sigma_(2, 2) = FusionEKF::inf_variance; Sigma_(3, 3) = FusionEKF::inf_variance;
	    // All done by the looks of it!
	  
	  }
	}
	  
	// The rest is just allocation of space... 
	// Transition matrix
	F_ = MatrixXd(4, 4);
	// Transition covariance (NOTE: I am reverting to Thrun's conventions: R for transition, Q for measurement)
	R_ = MatrixXd(4, 4);
	// Measurement Covariance(s) - In this case, 2
	vQ_ = { Cov_laser_, Cov_radar_ };
	// Measurement Jacobians
	vH_ = { MatrixXd::Zero(2, 4), MatrixXd::Zero(3, 4) };
	// fixing the laser matrix
	vH_[0](0, 0) = vH_[0](1, 1) = 1;
	// Done 
}


/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  // if this is the first reception, use the measurement to initialize the prior
  if (!is_initialized_) {
	// initialize the filter
	Init(measurement_pack);
	// forward timestamp cache
	previous_timestamp_ = measurement_pack.timestamp_;
	// raise the flag so that we skip this part next time
	is_initialized_ = true; 
	return;
  }

  // Compute time since last sample
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  // Forward sampling time 
  previous_timestamp_ = measurement_pack.timestamp_;

  // 1. Update the transition matrix
  F_ << 1, 0, dt, 0,
	0, 1, 0, dt,
	0, 0, 1, 0,
	0, 0, 0, 1;
  
  //2. Re-compute the transition covariance R using the "noisy acceleration" concept
  double dt4 = dt*dt*dt*dt, 
	 dt3 = dt*dt*dt,
	 dt2 = dt*dt;
  
  R_ <<   dt4 * noise_ax / 4, 0.0, dt3 * noise_ax / 2,  0.0,
	            0.0,   dt4 * noise_ay / 4.0, 0.0, dt3 * noise_ay / 2,
	            dt3 * noise_ax / 2,    0.0  ,   dt2*noise_ax, 0.0,
	            0.0 ,  dt3*noise_ay / 2, 0.0,   dt2*noise_ay;
	 
  //3. Get the posterior marginal over the previous state
  Predict();

  //4. Now update the posterior
  Update(measurement_pack);
  
  // print the moments of state distribution
  cout << "x_= " << x_ << endl;
  cout << "Sigma_= " << Sigma_ << endl;


 
}


// The prediction step
void FusionEKF::Predict() {
  // Get the new mean
  x_ = F_ * x_;
  // New covariance
  Sigma_ = F_ * Sigma_ * F_.transpose() + R_;
  // all done
}

// The Measurement update!
void FusionEKF::Update(const MeasurementPackage &pack) {
  
  // timeout for the Levenberg - Marquardt algorithm
  int timeout = 50;
  // do this here because we will need it...
  MatrixXd I4 = MatrixXd::Identity(x_.size(), x_.size()); // we will need this in either solution...
  
  // linear update if laser measurement
  if (pack.sensor_type_ == MeasurementPackage::LASER) {
    //return;
    //cout<<"A LIDAR measurement : "<<endl<<pack.raw_measurements_<<endl;
    if (pack.raw_measurements_.norm() < 0.0000001) return; // just skip it and rely on the prior
    // extracting the two measurement vectors
    VectorXd z_l = pack.raw_measurements_; // lidar measurement
    
    // Get a reference to the laser measurement covariance
    MatrixXd& Ql = vQ_[0];
    
    // 1. Lidar update
    MatrixXd &Hl = vH_[0];
  
    VectorXd y = z_l - Hl * x_;
    MatrixXd Hlt = Hl.transpose(); // transposition could be done once, but hey, its constant time anyway...
    MatrixXd S = Hl * Sigma_ * Hlt + Ql;
    // Now S can be non-invertible under extreme circumstances, but, 
    // since its a PSD matrix, we can make it invertible by adding 0.001*I4
    MatrixXd Sinv; // this one is almost guranteed (even if det == 0, it has a pseudo inverse for sure)
    if (S.determinant() == 0) Sinv = (S+0.001 * I4).inverse();
    else 
      Sinv = S.inverse();
    // "Kalman gain"...
    MatrixXd K = Sigma_ * Hlt * Sinv;
    // Finally, new state estimate and covariance
    x_ = x_ + (K * y);
    
    Sigma_ = (I4 - K * Hl) * Sigma_;
  } 
    else { // 2. This the Radar update. We do Gauss-Newton optimization
      //cout<<"A RADAR measurement : "<<endl<<pack.raw_measurements_<<endl;
      
      if (fabs(pack.raw_measurements_[0]) < 0.0000001) return; // just skip it and rely on the prior
      
      // get the measurement
      VectorXd z_r = pack.raw_measurements_;
      
      // making sure the angle is in [-pi, pi] range.
      double theta = z_r[1];
      //constexpr double PI  =3.141592653589793238463;
      if (theta > M_PI) {
	theta = -(2 * M_PI - theta);
	z_r[1] = theta;
      }
      
      VectorXd x0 = x_; // we need to cache the firt mean in order to use it as prior in every step of the G-N process
      MatrixXd Omega0;  // Similarly, Omega0 is used as the prior inverse covariance (Fisher information) in every step of the G-N process
      
      // Now inverting prior covariance, which normally should be invertible.
      // However, in the extreme case of zero-determinant, we add 0.001*I to make it invertible
      // This is equivalent to adding an extra regularization on the state vector. All being well, 
      // it should not make any difference in the end.
      if (Sigma_.determinant() == 0) Omega0 = (Sigma_ + (0.001*I4)).inverse();
	else 
	  Omega0 = Sigma_.inverse();
      
      // Now get a shortcut for the radar Jacobian
	// For the time being it contains garbage...
      MatrixXd &Hr = vH_[1];
      // and the radar measurement covariance
      MatrixXd &Qr = vQ_[1];
      
      // ******************* Now, doing Levenberg - Marquardt iteration ***********************
      // cache the inverse Qr
      MatrixXd Qinv = Qr.inverse();
      // evaluate the radar model at x_
      VectorXd h = MeasurementPackage::EvaluateRadarModel(x_);
      // compute squared error 
      double error_sq = (z_r - h).dot( Qinv * (z_r - h) ); // no need to add the prior because the difference would be zero of course...
      //double error_sq = (z_r - h).dot( z_r - h );
      double min_error_sq = error_sq;
      double diff_error_sq = 99999999.9;
      // CAUTION: Now preparing the Levenberg-Marquardt iteration
      double lambda = 0.0001; // LM regularization factor
      int step = 0;           // iteration step counter
      double tolerance = 10E-6;  // tolerance
      bool stop = min_error_sq < tolerance;   // stop flag
 
      // A cache vector for temporary state updates
      VectorXd x_temp;
      // a cache matrix for the inverse covariance (Fisher information) matrix storage.
      // We need this in order to assign covariance in the end
      MatrixXd Omega = Omega0; 
      //cout<<"Omega0 is : "<<endl<<Omega0;
      
      // looping to a better place hopefully...
      while (!stop) {
	
	// Get the Radar Jacobian for the current posterior
	// Now, in the case that we have zeros in the prior (possibly due to bad initialization),
	// we DONT CANT HAVE a zeros Jacobian, because it will take us NOWHERE,
	// So we move the state mean at the measurement and start the search from there...
	// (I doubt this will happpen with the test data, but just in case...)
	if (x_[0]*x_[0] + x_[1]*x_[1] < 0.0000001)  {
	  
	  x_[0] = z_r[0] * cos(z_r[1]);
	  x_[1] = z_r[0] * sin(z_r[1]);
	  x_[2] = z_r[2] * cos(z_r[1]);
	  x_[3] = z_r[2] * sin(z_r[1]);
	}
	
	MeasurementPackage::RadarJacobian(x_, Hr);
	//h = MeasurementPackage::EvaluateRadarModel(x_);
	
	bool improvement = false;
	// no need to recompute the Jacobian unless an improvement in the solution occured...
	while (!improvement && !stop) {
	  // Compute a new temporary information matrix
	  MatrixXd Omega_temp = Hr.transpose() * Qinv * Hr + Omega0 + lambda * I4;
	  //cout<<"Omega_temp is : "<<endl<<Omega_temp;
	  // Solve for the change in x, δx
	  VectorXd deltax = Omega_temp.inverse() * ( Hr.transpose() * Qinv * (z_r - h) + Omega0 * (x0 - x_) );
	
	  //cout<<"deltax is : "<<endl<<deltax;
	  // Now get the new temporary state
	  x_temp = x_ + deltax;
	
	  // Now, evaluate the radar model at x_temp
	  VectorXd h_temp = MeasurementPackage::EvaluateRadarModel(x_temp);
	
	  // now check the new error
	  double new_error_sq = (z_r - h_temp).dot( Qinv * (z_r - h_temp) ) + (x_temp - x0).dot( Omega0 * (x_temp - x0) );
	  //double error_sq = (z_r - h).dot( z_r - h );
	  if (new_error_sq < min_error_sq) { // great! We got somewhere better!
	 
	    x_ = x_temp;
	    h = h_temp;
	    Omega = Omega_temp/*-lambda*I4*/;
	    lambda /= 10;
	    improvement = true;
	    min_error_sq = new_error_sq;
	  }
	  else lambda *= 10;
	
	 diff_error_sq = fabs(new_error_sq - error_sq);
	 error_sq = new_error_sq;
	
	 step++;
	 stop = error_sq < tolerance || diff_error_sq < 10E-7 || step >= timeout;
	}
    }
    // Now we simply update the covariance matrix
    Sigma_ = Omega.inverse();
 }// end else
 
}


/*void FusionEKF::Update(const MeasurementPackage &pack) {
  
  // timeout for the Levenberg - Marquardt algorithm
  int timeout = 50;
  // do this here because we will need it...
  MatrixXd I4 = MatrixXd::Identity(x_.size(), x_.size()); // we will need this in either solution...
  
  // linear update if laser measurement
  if (pack.sensor_type_ == MeasurementPackage::LASER) {
    //return;
    //cout<<"A LIDAR measurement : "<<endl<<pack.raw_measurements_<<endl;
    if (pack.raw_measurements_.norm() < 0.0000001) return; // just skip it and rely on the prior
    // extracting the two measurement vectors
    VectorXd z_l = pack.raw_measurements_; // lidar measurement
    
    // Get a reference to the laser measurement covariance
    MatrixXd& Ql = vQ_[0];
    
    // 1. Lidar update
    MatrixXd &Hl = vH_[0];
  
    VectorXd y = z_l - Hl * x_;
    MatrixXd Hlt = Hl.transpose(); // transposition could be done once, but hey, its constant time anyway...
    MatrixXd S = Hl * Sigma_ * Hlt + Ql;
    // Now S can be non-invertible under extreme circumstances, but, 
    // since its a PSD matrix, we can make it invertible by adding 0.001*I4
    MatrixXd Sinv; // this one is almost guranteed (even if det == 0, it has a pseudo inverse for sure)
    if (S.determinant() == 0) Sinv = (S+0.001 * I4).inverse();
    else 
      Sinv = S.inverse();
    // "Kalman gain"...
    MatrixXd K = Sigma_ * Hlt * Sinv;
    // Finally, new state estimate and covariance
    x_ = x_ + (K * y);
    
    Sigma_ = (I4 - K * Hl) * Sigma_;
  } 
    else { // 2. This the Radar update. We do Gauss-Newton optimization
      //cout<<"A RADAR measurement : "<<endl<<pack.raw_measurements_<<endl;
      
      if (abs(pack.raw_measurements_[0]) < 0.0000001) return; // just skip it and rely on the prior
      
      // get the measurement
      VectorXd z_r = pack.raw_measurements_;
      
      // making sure the angle is in [-pi, pi] range.
      double theta = z_r[1];
      
      if (theta > M_PI) {
	theta = -(2 * M_PI - theta);
	z_r[1] = theta;
      }
      
      MatrixXd Q = Cov_radar_;
      // iinitial mean and variance
      VectorXd mu0 = x_;
      MatrixXd Sigma0 = Sigma_;
      VectorXd muk = mu0;
      MatrixXd Sigmak = Sigma0;
      
      // now looping 1 time-only....
      for (int i = 0; i<1; i++) {
	// evaluate the function at the current estimate
	VectorXd h = MeasurementPackage::EvaluateRadarModel(muk);
	// Get the Jacobian of the measurement model at muk
	MatrixXd Hk(3, 4);
	MeasurementPackage::RadarJacobian(muk, Hk);
	MatrixXd S = Hk * Sigma0 * Hk.transpose() + Q;
	MatrixXd K = Sigma0 * Hk.transpose() * S.inverse();
	muk = muk + K * (z_r - h - Hk * (mu0 - muk));
	Sigmak = (I4 - K * Hk)*Sigma0;
	
      }
      
      x_ = muk;
      Sigma_ = Sigmak;
      
 }// end else
 
}*/