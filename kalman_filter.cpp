//
//  kalman_filter.cpp
//  
//
//  Created by Adam Kilbourne on 2019-11-24.
//

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

#include "matrixmath.h"
#include "kalman_filter.hpp"

int main(int argc, const char * argv[]) {
    // 1. account for user inputs (filename, write to log/console, exit condition)
    // 2. read data from file
    // 3. format into state-space object
    // 4. format into KF obj
    // 5. run the filter until terminated
    
    cout<<argv[0]<<"\n";
    cout<<"Kalman Filter\n\n";
    cout<<"\n";
    vector<vector <int>> F{{0,1,2},{3,4,5},{6,7,8}};
    vector<vector <int>> G{{0,1,2},{3,4,5},{6,7,8}};
    vector<vector <int>> Z{{0,1,2,3},{3,4,5,6},{6,7,8,9}};
    vector<vector <int>> H;
    F[2][2] = 10;
    cout<<"\n";
    Matrix::print(F);
    Matrix::print(Z);
    H = Matrix::matSub(F, G);
    Matrix::print(H);
    cout<<"\n";
    H = Matrix::matMult(F, Z);
    Matrix::print(H);
    cout<<"\n";
    H = Matrix::TPose(F);
    Matrix::print(H);
    cout<<"\n";
    return 0;
}
