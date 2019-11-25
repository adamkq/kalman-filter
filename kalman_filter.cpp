//
//  kalman_filter.cpp
//  
//
//  Created by Adam Kilbourne on 2019-11-24.
//

#include <iostream>
#include <cmath>
using namespace std;

#include "kalman_filter.hpp"

int main(int argc, const char * argv[]) {
    // 1. account for user inputs (filename, log/console/neither, exit condition)
    // 2. read data from file
    // 3. format into state-space object
    // 4. format into KF obj
    // 5. run the filter until terminated
    
    cout<<argv[0]<<" "<<argv[1]<<"\n";
    cout<<"Kalman Filter\n\n";
    cout<<"\n";
    int F[3][3] = {{0,1,2},{3,4,5},{6,7,8}};
    cout<<F[1][1];
    cout<<"\n";
    return 0;
}
