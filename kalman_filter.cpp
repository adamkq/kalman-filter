//
//  kalman_filter.cpp
//  
//
//  Created by Adam Kilbourne on 2019-11-24.
//

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <iomanip>

using namespace std;

#include "matrixmath.h"
#include "kalman_filter.hpp"

int main(int argc, const char * argv[]) {
    cout<<"Kalman Filter\n";
    bool consoleFlag = false;
    bool loggingFlag = false;
    string logfile;
    ofstream f;
    
    for (int i = 0; i < argc; i++)
    {
        if (string(argv[i]) == "-c")
        {
            consoleFlag = true;
        }
        
        if (string(argv[i]) == "-f" && i < argc - 1)
        {
            logfile = string(argv[i + 1]);
            try
            {
                f.open(logfile);
                loggingFlag = true;
                cout<<"Logging to: "<<logfile<<"\n";
            }
            catch (...)
            {
                cout<<"A problem occurred when trying to open the specified log file. Data will not be logged.\n";
            }
        }
    }
    
    
    cout<<fixed;
    cout<<setprecision(3); // align tab-separated values with -sign
    
    double dt = 1.0/30; // timestep
    
    // n states
    // m <= n measurements (outputs)
    // k <= n controls (inputs)
    
    vector<vector <double>> A; // (n * n) state matrix
    vector<vector <double>> B; // (n * k) input matrix
    vector<vector <double>> C; // (m * n) output matrix
    
    vector<vector <double>> Q; // (n * n) process uncertainty matrix
    vector<vector <double>> P; // (n * n) estimation uncertainty
    vector<vector <double>> R; // (m * m) measurement uncertainty
    
    // projectile motion
    A = {
        {1, dt, 0},
        {0, 1, dt},
        {0, 0, 1}
    };
    
    // control acceleration only
    B = {
        {0},
        {0},
        {1}
    };
    
    // position and velocity observable
    C = {
        {1, 0, 0},
        {0, 1, 0}
    };
    
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
    
    R = {
        {5, 0},
        {0, 5}
    };
    
    // initial update. This is turned into a 2D vector within the KF.
    vector <double> xi = {1, 1, -9.81};
    vector <double> x_obs;
    
    // measurement and control. These are turned into a 2D vector within the KF. Control inputs are typically treated as exact (non-noisy).
    vector<double> measure;
    vector<double> control;
    
    // Noisy measurements
    vector<double> measurements = {
        1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
        1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
        2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
        2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
        2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
        2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
        2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
        1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
        0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
    };
    
    KalmanFilter kf(A, B, C, Q, P, R);
    kf.initialize(xi, dt);
    
    if (consoleFlag)
    {
        kf.repr();
        kf.printState();
    }

    for (int i = 0; i < min(100, int(measurements.size())); i++)
    {
        // to demonstrate multiple measurements, the velocity is being 'measured' as a linear extrapolation from xi
        measure = {measurements[i], xi[1] + xi[2]*kf.getTime()};
        
        // ramp input 'thrust' for demonstration
        control = {0.0 * kf.getTime()};
        
        kf.update(measure, control);
        if (consoleFlag)
        {
            kf.printState();
        }
        x_obs = kf.getState(); // will use for logging
    }
    
    if (loggingFlag)
    {
        f.close();
    }
    // cout<<"Kalman Filter stopped after "<<kf.getTime()<<" seconds simulated.";
    cout<<"\n";
    return 0;
}
