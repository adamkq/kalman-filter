//
//  kalman_filter.hpp
//  
//
//  Created by Adam Kilbourne on 2019-11-24.
//

#ifndef kalman_filter_hpp
#define kalman_filter_hpp

#include <stdio.h>

class KalmanFilter
{
    // Fixed Matrices + K.
    vector<vector <double>> A, B, C, Q, P0, R, K;
    // State Matrices.
    vector<vector <double>> x_pre, P_pre, x_obs, P_obs;
    // Current time and timestep.
    double t, dt;

public:
    KalmanFilter(
        vector<vector <double>> A, // state matrix
        vector<vector <double>> B, // input matrix
        vector<vector <double>> C, // output matrix
        vector<vector <double>> Q, // process uncertainty matrix
        vector<vector <double>> P, // estimation uncertainty
        vector<vector <double>> R // measurement uncertainty
    );
    /// Print the fixed matrices of the filter.
    void repr();
    /// Initialize the filter with a best guess.
    void initialize(vector <double> xi, double dt);
    /// Print the time and state to the console in row-form.
    vector <double> getState();
    /// Return elapsed time.
    double getTime();
    /// Increment the KF to the next state.
    void update(vector <double> y, vector <double> u);
};

KalmanFilter::KalmanFilter(
        vector<vector <double>> A,
        vector<vector <double>> B,
        vector<vector <double>> C,
        vector<vector <double>> Q,
        vector<vector <double>> P,
        vector<vector <double>> R
         ): A(A),
            B(B),
            C(C),
            Q(Q),
            P0(P),
            R(R),
            x_pre(1, vector<double>(A.size())),
            x_obs(1, vector<double>(A.size())),
            P_pre(P0),
            P_obs(P0)
            
{
    this->A = A;
    this->B = B;
    this->C = C;
    this->Q = Q;
    this->P0 = P;
    this->R = R;
}

void KalmanFilter::repr()
{
    cout<<"\nA:\n";
    Matrix::print(A);
    cout<<"\nB:\n";
    Matrix::print(B);
    cout<<"\nC:\n";
    Matrix::print(C);
    cout<<"\nQ:\n";
    Matrix::print(Q);
    cout<<"\nP0:\n";
    Matrix::print(P0);
    cout<<"\nR:\n";
    Matrix::print(R);
    cout<<"\nK:\n";
    Matrix::print(K);
    cout<<"\n---\n";
}

vector <double> KalmanFilter::getState()
{
    return Matrix::tpose(x_obs)[0];
}

double KalmanFilter::getTime()
{
    return t;
}

void KalmanFilter::initialize(vector <double> xi, double dt)
{
    this->x_obs = Matrix::tpose({xi});
    this->P_obs = P0;
    this->dt = dt;
    this->t = 0;
}

void KalmanFilter::update(vector <double> measure, vector <double> control)
{
    vector<vector <double>> y = Matrix::tpose({measure});
    vector<vector <double>> u = Matrix::tpose({control});
    vector<vector <double>> Inv;
    vector<vector <double>> noise;
           
    // predict state
    x_pre = Matrix::add(Matrix::mult(A, x_obs), Matrix::mult(B, u));
    P_pre = Matrix::add(Matrix::mult(A, P_obs, Matrix::tpose(A)), Q);
                
    // update K
    Inv = Matrix::add(Matrix::mult(C, P_pre, Matrix::tpose(C)), R);
    Inv = Matrix::rowReduce(Inv, Matrix::iden(Inv.size()));

    if (Matrix::hasNaNInf(Inv))
    {
        throw runtime_error("Kalman Gain Non-Invertible.");
    }
                            
    K = Matrix::mult(P_pre, Matrix::tpose(C), Inv);
    
    // update state
    // y = Normal(C * x_obs, R)
    noise = Matrix::subtract(y, Matrix::mult(C, x_pre));
    x_obs = Matrix::add(x_pre, Matrix::mult(K, noise));
    P_obs = Matrix::subtract(P_pre, Matrix::mult(K, C, P_pre));

    // update time
    t += dt;
}

#endif /* kalman_filter_hpp */
