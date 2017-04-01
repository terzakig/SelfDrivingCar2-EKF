# AN EKF filter for pedestrian tracking using LIDAR and RADAR processed measurements

## Code Structure and minor functional modifications
I have turned the `KalmanFilter` class essentially into an interface, so now all of its methods (except the constructor and destructor) are _virtual_ and the class now is stored in a single header file. The `FusionEKF` class now is derived from `KalmanFiletr` and it implements methods `predict` and `update`. The state mean `x_`, state covariance matrix `Sigma_`, transition model covariance `R_` as well as the list of measurement meatrices `vH` and respective measurement covariance matrices `vQ` are all declared in `KalmanFilter` and can be accessed directly as fields inside `FusionEKF`.
  New patragraph?

