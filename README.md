Implement a Kalman Filter in C++.

The variables are defined as:

PREDICTION:

x_pre: predicted state vector (n * 1)
P_pre: predicted covariance matrix (n * n)
A: state matrix (based off of physical model) (n * n)
u: control vector (k * 1)
B: control matrix (usually identity with 1's corresponding to the controllable states) (n * k)
Q: process uncertainty matrix (turbulence) (n * n)

UPDATE:

x_obs: updated state vector (n * 1)
P_obs: updated cov. matrix (n * n)
z:  sensor reading vector (m * 1)
C: output matrix (usually identity with 1's corresponding to the observable states) (m * n)
R: measurement noise cov (m * m)
K: kalman gain matrix (determined from C, P, R) (n * m)


