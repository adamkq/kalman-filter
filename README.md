Implement a Kalman Filter in C++

The variables are defined as:
STATE:
x: predicted vector
x': updated vector
P: predicted covariance matrix
P': updated CM

PREDICTION:
F: prediction matrix (based off of physical model)
u: control vector (read as 0, step, ramp, or sine)
B: control matrix
Q: uncertainty matrix (turbulence)

UPDATE:
H: sensor matrix
R: noise matrix (used only to find K) (may not include this)
z:  sensor reading vector
K: kalman gain matrix (determined from H, P, R)

operations: add, subtract, mult, inv (det and adj), tpose, normalize.

Matrices can be implemented as 2D arrays. 
