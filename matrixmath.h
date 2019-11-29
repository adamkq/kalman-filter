//
//  matrixmath.h
//  
//
//  Created by Adam Kilbourne on 2019-11-25.
//

#ifndef matrixmath_h
#define matrixmath_h


class Matrix{
    public:
        static void print(vector<vector <double>> A) //print the matrix.
        {
           for (int i=0; i<A.size(); i++)
           {
               for (int j=0; j<A[0].size(); j++)
               {
                   cout << A[i][j] << " ";
               }
               cout << endl;
           }
        }
    
        static vector<vector <double>> matAdd(vector<vector <double>> A, vector<vector <double>> B)
        {
            if (A.size() != B.size() || A[0].size() != B[0].size())
            {
                throw invalid_argument(" Matrix dimensions not equal.");
            }
            vector<vector <double>> C;
            for (int i=0; i<A.size(); i++)
            {
                vector <double> row;
                for (int j=0; j<A[0].size(); j++)
                {
                    row.push_back(A[i][j] + B[i][j]);
                }
                C.push_back(row);
            }
            return C;
        }
    
        static vector<vector <double>> matSub(vector<vector <double>> A, vector<vector <double>> B)
            // for semantic clarity, this is a separate function from matAdd
        {
            if (A.size() != B.size() || A[0].size() != B[0].size())
            {
                throw invalid_argument(" Matrix dimensions not equal.");
            }
            vector<vector <double>> C;
            for (int i=0; i<A.size(); i++)
            {
                vector <double> row;
                for (int j=0; j<A[0].size(); j++)
                {
                    row.push_back(A[i][j] - B[i][j]);
                }
                C.push_back(row);
            }
            return C;
        }
    
        static vector<vector <double>> matMult(vector<vector <double>> A, vector<vector <double>> B)
        {
            if (A[0].size() != B.size()) // Matrices not square
            {
                throw invalid_argument("Matrix dimensions A and B cannot multiply.");
            }
            vector<vector <double>> C;
            double elem;
            
            for (int i=0; i<A.size(); i++)
            {
                vector <double> row;
                for (int j=0; j<B[0].size(); j++)
                {
                    elem = 0;
                    for (int k=0; k<B.size(); k++)
                    {
                        elem += A[i][k] * B[k][j];
                    }
                    row.push_back(elem);
                }
                C.push_back(row);
            }
            return C;
        }
    
        static vector<vector <double>> TPose(vector<vector <double>> A)
        {
            vector<vector <double>> C(A[0].size(), vector <double> (A.size()));
            for (int i=0; i<A.size(); i++)
            {
                vector <int> row;
                for (int j=0; j<A[0].size(); j++)
                {
                    C[j][i] = A[i][j];
                }
            }
            return C;
        }
    
        static vector<vector <double>> matId(int n)
        {
            vector<vector <double>> Iden(n, vector <double> (n));
            for (int i = 0; i < n; i++)
            {
                Iden[i][i] = 1.0;
            }
            return Iden;
        }
    
        // === Invert Matrix ===
        
        static vector<vector <double>> rowReduce(vector<vector <double>> A, vector<vector <double>> C)
        {
            // Attempts to solve AB = C for B through Gaussian methods (precond. all mats square)
            // If C is an Id. matrix, B will be equal to inv(A)
            // If a div by zero condition is encountered, the output matrix will contain inf or NaN values. These will be checked by the KF.
            // As designed, the KF only performs an inversion when it initializes (to find the Kalman Gain), so this fcn only has to be called once, not on every timestep.
            
            int N = A.size();
            double leadCoeff, rowCoeff;
            
            // Checks if A has more rows than C. A may have fewer rows than C.
            if (N > C.size())
            {
                throw invalid_argument("Matrix to be reduced (A) must have the same number of rows or fewer as its product (C).");
            }
            
            // degenerate case
            if (N == 1)
            {
                C[0][0] = C[0][0]/A[0][0];
                return C;
            }
            
            // row echelon
            for (int i = 0; i < N; i++)
            {
                leadCoeff = A[i][i]; // diagonal element
                for (int j = i + 1; j < N; j++)
                {
                    rowCoeff = -A[j][i]/leadCoeff; // cancel elements in column
                    for (int k = i; k < N; k++)
                    {
                        A[j][k] += A[i][k] * rowCoeff;
                        C[j][k] += C[i][k] * rowCoeff;
                    }
                }
            }
            
            cout<<"row echelon Finished. A and C Shown:\n";
            print(A);cout<<"\n";
            print(C);cout<<"\n";
            
            // reduced row
            for (int i = N-1; i > 0; i--)
            {
                leadCoeff = A[i][i]; // diagonal element
                for (int j = i - 1; j >= 0; j--)
                {
                    rowCoeff = -A[j][i]/leadCoeff; // cancel elements in column
                    for (int k = 0; k < N; k++)
                    {
                        A[j][k] += A[i][k] * rowCoeff;
                        C[j][k] += C[i][k] * rowCoeff;
                        
                    }
                }
                for (int k = 0; k < N; k++) // scale row. May assign negative zeros
                {
                    A[i][k] = A[i][k] / leadCoeff; // should always be 1 on diagonal, 0 elsewhere
                    C[i][k] = C[i][k] / leadCoeff;
                }
            }
            
            cout<<"rowReduce Finished. A and C Shown: \n";
            print(A);cout<<"\n";
            print(C);cout<<"\n";
            return C;
        }
    
        
};

#endif /* matrixmath_h */
