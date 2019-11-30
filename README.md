Implement a Kalman Filter in C++

The variables are defined as:
STATE:
x: predicted vector
x': updated vector
P: predicted covariance matrix
P': updated CM

PREDICTION:
A: state matrix (based off of physical model) (n * n)
u: control vector (0, step, ramp, or sine for each controllable state) (1 * m)
B: control matrix (usually identity with 1's corresponding to the controllable states)
Q: Process Noise matrix (turbulence)

UPDATE:
C: output matrix (usually identity with 1's corresponding to the observable states)
R: measurement noise cov (used only to find K at initialization)
z:  sensor reading vector (may get from random function instead of pre-defined vector)
K: kalman gain matrix (determined from C, P, R)

operations: add, subtract, mult, inv (det and adj), tpose, normalize.

Matrices can be implemented as 2D arrays. 

git | illustration
A | F
P, Q, and R are the same
C | H

