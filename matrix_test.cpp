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
    vector<vector <double>> single1{{3}};
    vector<vector <double>> single2{{5}};
    vector<vector <double>> A{
        {1.1,2,3},
        {4,5,6},
        {7,8,7}
    };
    vector<vector <double>> B{
        {1,2,3,4},
        {-2,5,1,-3},
        {7,8,6,-5},
        {0.5,0,0,1}
    };
    
    cout<<"Subtraction \n";
    Matrix::print(F);cout<<"\n";
    Matrix::print(G);cout<<"\n";
    H = Matrix::subtract(F, G);
    Matrix::print(H);cout<<"\n";
    
    cout<<"Multiplication of 2 mats, different dims\n";
    Matrix::print(F);cout<<"\n";
    Matrix::print(Z);cout<<"\n";
    H = Matrix::mult(F, Z);
    Matrix::print(H);cout<<"\n";
    
    cout<<"Multiplication of 4 mats, same dims\n";
    H = Matrix::mult(F, G, Matrix::iden(F.size(),2), Matrix::iden(F.size(),3));
    Matrix::print(H);cout<<"\n";
    
    cout<<"tpose \n";
    Matrix::print(Z);cout<<"\n";
    H = Matrix::tpose(Z);
    Matrix::print(H);cout<<"\n";
    
    cout<<"Identity \n";
    H = Matrix::iden(8);
    Matrix::print(H);cout<<"\n";
    
    cout<<"Row Reduce 1x1\n";
    H = Matrix::rowReduce(single1, single2);
    Matrix::print(H);cout<<"\n";
    
    cout<<"Row Reduce 3x3\n";
    H = Matrix::rowReduce(A, F);
    Matrix::print(H);cout<<"\n";
    
    cout<<"Row Reduce Invert 4x4\n";
    H = Matrix::rowReduce(B, Matrix::iden(B.size()));
    Matrix::print(H);cout<<"\n";
    
    cout<<"Row Reduce Invert 2x2 Singular\n";
    H = Matrix::rowReduce({{2,4},{3,6}}, Matrix::iden(2));
    Matrix::print(H);cout<<"\n";
    
    cout<<"\n";
}
