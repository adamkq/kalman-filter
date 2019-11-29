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
    // 2. read data from file
    // 3. format into state-space object
    // 4. format into KF obj
    // 5. run the filter until terminated
    
    int a = 1;
    float b = 5.4;
    cout<<argv[0]<<"\n";
    cout<<"Kalman Filter\n\n";
    cout<<a*b;
    cout<<"\n";
    return 0;
}
