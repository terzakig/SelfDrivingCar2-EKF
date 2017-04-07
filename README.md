# AN EKF filter for position tracking using LIDAR and RADAR processed measurements

## Code Structure and minor functional modifications
I have turned the `KalmanFilter` class essentially into an interface, so now all of its methods (except the constructor and destructor) are _virtual_ and the class now is stored in a single header file. The `FusionEKF` class now is derived from `KalmanFiletr` and it implements methods `predict` and `update`. The state mean `x_`, state covariance matrix `Sigma_`, transition model covariance `R_` as well as the list of measurement meatrices `vH` and respective measurement covariance matrices `vQ` are all declared in `KalmanFilter` and can be accessed directly as fields inside `FusionEKF`.

The `tools` class is now a namespace and contains only the `CalculateRMSE` and the Jacobian became a static member of the `MeasurementPackage` class. The class also now has an additional static method that evaluates the radar measurement function in from given position and velocity.
### Compiling and Executing
Compling and running should be straightforward (I have included the test files in the build directory:
```

cd build
cmake ..
make
./ExtendedKF sample-laser-radar-measurement-data-1.txt output1.txt
./ExtendedKF sample-laser-radar-measurement-data-2.txt output2.txt
```
## How the filter works
The filter processes the LIDAR and RADAR measurements interchangeably with the same update function. The prediction is straightforward using a simple motion model involving position and velocity in a 4D state vector. The entire measurement package is passed-on to`update()`. If the origin of the measurement is the _LIDAR_, the filter performs a simple linear update to the posterior. If however, the data come from the _RADAR_, then things are a bit different.
### RADAR Measurement Updates
Radar measurement updates are done iteratively. The update method runs a **Levenberg-Marquardt** iterative process starting from the current state mean`x_` and iterates until a convergence condition is met. In practice, measurements are fairly good and the algorithm does not take more than 3-4 iterations to refine the estimate. The LM algorithm does not use the usual KF formulas (i.e., the ones using the Kalman gain), as they are not necessary. The Kalman gain is a shortcut in the posterior computation that spares the computation of an extra matrix inverse (i.e., the inverse of the predicted state covariance). In this case hwoever, there is no need to do so, as there will be several iterations and the number of matrix inversions will not be affected if we use the Kalman gain.
### The "Zero-Measurements"
Now, these "zero-position" measurements don't come with much information about them, but I am regarding them as degenerate and potentially detrimental measurements for many reasons. The main reason being, that when a "zero-position" measurement arrives from the RADAR, then the Jacobian is zero or, in the case of the range derivative, undetermined and there can be no improvement in the posterior, although the covariance will decrease if we employ the standard formulas (the LM algorithm will also exit without an update in the state, but with a brand new covariance unfortunately), which will make the filter moree confident about a false estimate. There are two cases in terms of receiving zero measurements:

* ___Zero-Measurement on Initiazation___: In this case, regardless of data origin, the prior is initialized at [0, 0, 0, 0] with an "infinite" variance (diagonal matrix), which is basically a very large number (e.g., 10000). 
* ___Zero-Measurement on Update___: The zero-measurement is "dodgy" in many ways and for this reason we skip the update altogether. 
