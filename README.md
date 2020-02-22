# Purpose

Implement a Kalman Filter in C++. A Kalman filter is a popular state estimation algorithm, which are used to estimate parameters of a system that are otherwise difficult or impossible to measure. It models system behaviour by using a set of differential equations.

# Definition

The Kalman Filter has many variants but this project uses the standard equations for a KF. The variables are defined as:

### PREDICTION:

x_pre: predicted state vector (n * 1)
P_pre: predicted covariance matrix (n * n)
A: state matrix (based off of physical model) (n * n)
u: control vector (k * 1)
B: control matrix (usually identity with 1's corresponding to the controllable states) (n * k)
Q: process uncertainty matrix (turbulence) (n * n)

### UPDATE:

x_obs: updated state vector (n * 1)
P_obs: updated cov. matrix (n * n)
z:  sensor reading vector (m * 1)
C: output matrix (usually identity with 1's corresponding to the observable states) (m * n)
R: measurement noise cov (m * m)
K: kalman gain matrix (determined from C, P, R) (n * m)

# How To Install

There are no dependencies to any third-party libraries. Project should be ready to go out of the box.
