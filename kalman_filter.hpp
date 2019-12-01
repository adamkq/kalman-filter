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
    void printState();
    /// Return x_obs as 1D vector.
    vector <double> getState();
    /// Return elapsed time.
    double getTime();
    /// Update.
    void update(vector <double> y, vector <double> u);
};

KalmanFilter::KalmanFilter(
        vector<vector <double>> A,
        vector<vector <double>> B,
        vector<vector <double>> C,
        vector<vector <double>> Q,
        vector<vector <double>> P,
        vector<vector <double>> R
         ): A(A.size(), vector<double> (A[0].size())),
            B(B.size(), vector<double> (B[0].size())),
            C(C.size(), vector<double> (C[0].size())),
            Q(Q.size(), vector<double> (Q[0].size())),
            P0(P.size(), vector<double> (P[0].size())),
            R(R.size(), vector<double> (R[0].size())),
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
}

void KalmanFilter::printState()
{
    if (t == 0){
        cout<<"\nTime:\t\tStates ("<<x_obs.size()<<"):"<<"\n";
    }
    
    cout<<t<<"\t\t";
    for (int i = 0; i < x_obs.size(); i++)
    {
        cout<<x_obs[i][0]<<"\t";
    }
    cout<<"\n";
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
                
    K = Matrix::mult(P_pre, Matrix::tpose(C), Inv);

    // update state
    noise = Matrix::subtract(y, Matrix::mult(C, x_pre));
    x_obs = Matrix::add(x_pre, Matrix::mult(K, noise));
    P_obs = Matrix::subtract(P_pre, Matrix::mult(K, C, P_pre));

    // update time
    t += dt;
}

#endif /* kalman_filter_hpp */
