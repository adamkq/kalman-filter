//
//  kalman_filter.cpp
//  
//
//  Created by Adam Kilbourne on 2019-11-24.
//

#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <map>

using namespace std;

#include "matrixmath.h"
#include "kalman_filter.hpp"

int main(int argc, const char * argv[]) {
    // 1. account for user inputs (filename, write to log/console, exit condition)
    // 2. initialize matrices internally
    // 3. format into state-space object/ KF
    // 4. run the filter until terminated
    cout<<"Kalman Filter\n";
    cout<<argv[0]<<"\n";
    
    double dt = 1.0/30; // timestep
    
    int n = 3; // states
    int m = 3; // measurements (outputs)
    int k = 2; // controls (inputs)
    
    vector<vector <double>> A; // (n * n) state matrix
    vector<vector <double>> B; // (n * k) input matrix
    vector<vector <double>> C; // (m * n) output matrix
    
    vector<vector <double>> Q; // (n * n) process uncertainty matrix
    vector<vector <double>> P; // (n * n) estimate error
    vector<vector <double>> R; // (m * m) measurement uncertainty
    
    
    
    // projectile motion
    A = {
        {1, dt, 0},
        {0, 1, dt},
        {0, 0, 1}
    };
    
    // control velocity and acceleration
    B = {
        {0, 0},
        {1, 0},
        {0, 1}
    };
    
    // entirely observable
    C = Matrix::matId(m, 1);
    
    // suitable values for Q, P, and R
    Q = {
        {.05, .05, .0},
        {.05, .05, .0},
        {.0, .0, .0}
    };
    
    P = {
        {.1, .1, .1},
        {.1, 10000, 10},
        {.1, 10, 100}
    };
    
    R = Matrix::matId(m, 5);
    
    KalmanFilter kf(A, B, C, Q, P, R);
    kf.repr();

    cout<<"\n";
    return 0;
}
