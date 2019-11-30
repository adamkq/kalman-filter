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
    vector<vector <double>> A, B, C, Q, P, R, K;

public:
    KalmanFilter(
        vector<vector <double>> Ain, // state matrix
        vector<vector <double>> Bin, // input matrix
        vector<vector <double>> Cin, // output matrix
        vector<vector <double>> Qin, // process uncertainty matrix
        vector<vector <double>> Pin, // estimate error
        vector<vector <double>> Rin // measurement uncertainty
    );
    void repr();
};

KalmanFilter::KalmanFilter(
        vector<vector <double>> Ain,
        vector<vector <double>> Bin,
        vector<vector <double>> Cin,
        vector<vector <double>> Qin,
        vector<vector <double>> Pin,
        vector<vector <double>> Rin
         ): A(Ain.size(), vector<double> (Ain[0].size())),
            B(Bin.size(), vector<double> (Bin[0].size())),
            C(Cin.size(), vector<double> (Cin[0].size())),
            Q(Qin.size(), vector<double> (Qin[0].size())),
            P(Pin.size(), vector<double> (Pin[0].size())),
            R(Rin.size(), vector<double> (Rin[0].size()))
{
    A = Ain;
    B = Bin;
    C = Cin;
    Q = Qin;
    P = Pin;
    R = Rin;
    K = Matrix::matAdd(P, R);
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
    cout<<"\nP:\n";
    Matrix::print(P);
    cout<<"\nR:\n";
    Matrix::print(R);
    cout<<"\nK:\n";
    Matrix::print(K);
}

#endif /* kalman_filter_hpp */
