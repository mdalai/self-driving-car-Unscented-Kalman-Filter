# self-driving-car-Unscented-Kalman-Filter
Udacity Self Driving Car Nanodegree Term2 Project 2
## Why UKF
The [EKF(Extended Kalman Filter)](https://github.com/mdalai/self-driving-car-Extended-Kalman-Filter) has following issues:
- It works poor when the object is making turn. The prediction of object position in turning situation tends to be inaccurate because EKF adapted linear algorithm. 
- Non-linear to linear transformation is complicated in EKF. It adapted Jacobian matrix.

UKF works better dealing above issues. We will prove this with this project.


[//]: # (Image References)
[ukf_process]: ./assets/UKF_process.PNG
[pxpy_div0]: ./assets/pxpy_div0.PNG
[err_updateLaser]: ./assets/err_updateLaser.PNG
[ukf_nis1]: ./assets/ukf_nis1.PNG
[ukf_rmse1]: ./assets/ukf_rmse1.PNG
[ukf_ekf_compare]: ./assets/ukf_ekf_compare.PNG


## The UKF Intro
In order to improve accuracy when the object is turning, the UKF added turning parameters. The state is represented with following 5 dimensions:
- Px: position in X direction.
- Py: position in Y direction.
- V: velocity.
- yaw: turning angle.
- yaw rate: truning angle rate.

### The UKF takes following steps:
**Initialization**:
- The state and covarience need to be initialized. The initialization values impact the results in certain degree.
- Constant parameters need to be intialized. For example, process noise, measurement noise etc. These parameters are required to tune based on actual results. They impact tracking accuracy significantly.

**Prediction**:
- **Generate sigma points** based on current state and covarience. The augmented sigma points include two noises respectively _Longitude acceleration noise_ and _Yaw rate acceleration noise_. Therefore it has 7 demensions. We need to generate 15 sigma points.
- **Predict sigma points**: apply linear function to generated sigma points and get new sigma points.
- **Predict state mean and covarience**: based on the predicted sigma points calculates the new state and covarience.

**Update**:
- **Predict measurement state and covarience**: use predicted sigma points and transform it to Polar space in RADAR. Then calculate mean and covarience. 
- **Update**: Use the predicted mean and covarience, compare with measurement value, and update the state and covarience accordingly.

**UKF process**:

   ![alt text][ukf_process]


## The UKF in C++
- UKF code defines a lot of variables. Be careful with each one of them. I just misused variable names. For instance, Xsig_pred vs. Xsig_pred_.
- New added variables and methods have to be declared in the class definition part.
- But the object in the simulator ran only one step and stopped. The printed values are strange. The updated state vector x_ and state covariance P_  has many _nan_ values. To find the reason for this error, I decided to print values one by one.
  - I printed the Xsig_aug, no nan values in it. 
  - I printed the Xsig_pred, no nan values in it. 
  - I printed predicted x_, P_, found no nan values.
  - I printed Zsig, it has _nan_ values in its 3rd row. It means following code has problem: ```Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);```. I added a division by zero check as below. The problem solved.
    ```c++
         if (sqrt(p_x*p_x + p_y*p_y) > 0.0001){
           Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);
         } else{
           Zsig(2,i) = (p_x*v1 + p_y*v2 ) / 0.0001;
         }
     ```
- Then, the object in simulator ran for little while and stopped again. I checked the RMSE, it is way too high. I updated the following process noise parameters:
  ```c++
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5; // 30;
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.9; // 30;
  ```

- The object in simulator ran a while and stopped. The RMSE is still high. I changed the initialization value from 1 to 0. This did not help much. The simulator stopped and RMSE is still high.
  ```c++
      float v = 0;
      float yaw = 0;
      float yaw_r = 0;
  ```
- I decided to add the Laser processing part. The result get better, but same problem occurs again. The simulator stopped after while. The RMSE is high.  
- I found the problem finally. The x_, P_ have to be set to 0 every time I ran the state and covariance matrix prediction, as following:
   ```c++
       /**************************************************************************************************
        *  Predict the state and covariance matrix
        **************************************************************************************************/
       //predict state vector
       x_.fill(0.0);  // !!!!!! remember to set it to 0 every time !!!!!!!!
       for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
         x_ = x_+ weights_(i) * Xsig_pred_.col(i);
       }

       //predict state covariance matrix
       P_.fill(0.0); // !!!!!! remember to set it to 0 every time !!!!!!!!
       for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

         // state difference
         VectorXd x_diff = Xsig_pred_.col(i) - x_;
         //angle normalization
         while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
         while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

         P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
       }
   ```

## Evaluation
### Data collection
 - Update main.cpp to make it possible to collect data: 
 - Following data are collected: x_true, y_true, vx_true, vy_true, x_est, y_est, vx_est, vy_est	sensor, nis, RMSE_x, RMSE_y, RMSE_vx;
 - Run code. All values will be output in a CSV file.
   ```sh
      cmake .. && make                # compile the code
      ./UnscentedKF ukf_output.csv    # run the code
   ```
### NIS (Normalized Innovation Squared)
**Notes**: 95% of NIS values should be within 7.8 line.

   ![alt text][ukf_nis1]

### RMSE meet the standard
**Rubric Requirement**: your px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30].

   ![alt text][ukf_rmse1]

## UKF vs. EKF

 ![alt text][ukf_ekf_compare]
