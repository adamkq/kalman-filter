Implement a Kalman Filter in C++

The variables are defined as:

PREDICTION:

x_pre: predicted state vector
P_pre: predicted covariance matrix
A: state matrix (based off of physical model)
u: control vector
B: control matrix (usually identity with 1's corresponding to the controllable states)
Q: process uncertainty matrix (turbulence)

UPDATE:

x_obs: updated state vector
P_obs: updated cov. matrix
z:  sensor reading vector (may get from random function instead of pre-defined vector)
C: output matrix (usually identity with 1's corresponding to the observable states)
R: measurement noise cov (used only to find K at initialization)
K: kalman gain matrix (determined from C, P, R)


