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



## Evaluation
### RMSE meet the standard
**Standard**: your px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30].

### NIS

## UKF vs. EKF
### Tracking pic

### RMSE Plotting

