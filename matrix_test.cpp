//
//  matrix_test.cpp
//  
//
//  Created by Adam Kilbourne on 2019-11-28.
//

#include <iostream>
#include <stdio.h>
#include <vector>

using namespace std;

#include "matrixmath.h"

int main(int argc, const char * argv[]) {
    vector<vector <double>> F{{1,1,2.1},{3,8,5},{6,7,10}};
    vector<vector <double>> G{{2,3,2.2},{3,4,5},{6,7,8}};
    vector<vector <double>> Z{{1,1,2,3},{3,4,5,6},{6,7,8,9}};
    vector<vector <double>> H;
    vector<vector <double>> I4; // Identity
    vector<vector <double>> single1{{3}};
    vector<vector <double>> single2{{5}};
    vector<vector <double>> A{
        {1.1,2,3},
        {4,5,6},
        {7,8,7},
    };
    
    cout<<"Subtraction \n";
    Matrix::print(F);cout<<"\n";
    Matrix::print(G);cout<<"\n";
    H = Matrix::matSub(F, G);
    Matrix::print(H);cout<<"\n";
    cout<<"Multiplication \n";
    Matrix::print(F);cout<<"\n";
    Matrix::print(Z);cout<<"\n";
    H = Matrix::matMult(F, Z);
    Matrix::print(H);cout<<"\n";
    cout<<"TPose \n";
    Matrix::print(Z);cout<<"\n";
    H = Matrix::TPose(Z);
    Matrix::print(H);cout<<"\n";
    cout<<"Identity \n";
    H = Matrix::matId(8);
    Matrix::print(H);cout<<"\n";
    
    cout<<"Row Reduce \n";
    H = Matrix::rowReduce(A, Matrix::matId(A.size()));
    Matrix::print(H);cout<<"\n";
    Matrix::print(A);cout<<"\n";
    H = Matrix::rowReduce(single1, single2);
    Matrix::print(H);cout<<"\n";
}
