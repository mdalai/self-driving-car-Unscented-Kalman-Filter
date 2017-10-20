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



## Evaluation
### RMSE meet the standard
Standard:Your px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30].

### NIS

## UKF vs. EKF
### Tracking pic

### RMSE Plotting

