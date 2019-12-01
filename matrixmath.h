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
        /// Print the matrix.
        static void print(vector<vector <double>> A)
        {
           for (int i=0; i<A.size(); i++)
           {
               for (int j=0; j<A[i].size(); j++)
               {
                   cout<<A[i][j]<<"\t";
               }
               cout<<endl;
           }
        }
    
        /// Add mats.
        static vector<vector <double>> add(vector<vector <double>> A, vector<vector <double>> B)
        {
            if (A.size() != B.size() || A[0].size() != B[0].size())
            {
                throw runtime_error(" Matrix dimensions not equal.");
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
    
        /// Subtract mats.
        static vector<vector <double>> subtract(vector<vector <double>> A, vector<vector <double>> B)
            // for semantic clarity, this is a separate function from matAdd
        {
            if (A.size() != B.size() || A[0].size() != B[0].size())
            {
                throw runtime_error(" Matrix dimensions not equal.");
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
    
        /// Multiply 2 mats. This represents a base case for the variadic function below.
        static vector<vector <double>> mult(vector<vector <double>> A, vector<vector <double>> B)
        {
            if (A[0].size() != B.size()) // Matrices not square
            {
                throw runtime_error("Matrix dimensions A and B cannot multiply.");
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
    
        /// Multiply 3+ mats according to A(BC) = (AB)C. Variadic and recursive.
        template <typename T, typename... Args>
        static vector<vector <double>> mult(T mat1, Args... mat2)
        {
            return mult(mat1, mult(mat2...));
        }
    
        /// Transpose a matrix.
        static vector<vector <double>> tpose(vector<vector <double>> A)
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

        /// Return an identity matrix with a user-specified value on the diagonal.
        static vector<vector <double>> iden(int n, double entry = 1)
        {
            vector<vector <double>> I(n, vector <double> (n));
            for (int i = 0; i < n; i++)
            {
                I[i][i] = entry;
            }
            return I;
        }
    
        /**
         * Attempts to solve AB = C for B through Gaussian methods.
         * If C is an Id. matrix, B will be equal to inv(A)
         * If a div by zero condition is encountered, the output matrix will contain inf or NaN values. These will be checked by the KF.
         */
        static vector<vector <double>> rowReduce(vector<vector <double>> A, vector<vector <double>> C)
        {
            int N = A.size();
            double leadCoeff, rowCoeff;
            
            // Checks if A has more rows than C. A may have fewer rows than C.
            if (N > C.size())
            {
                throw runtime_error("Matrix to be reduced (A) must have the same number of rows or fewer as its product (C).");
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
                    for (int k = 0; k < N; k++)
                    {
                        A[j][k] += A[i][k] * rowCoeff;
                        C[j][k] += C[i][k] * rowCoeff;
                    }
                }
            }
            
            // reduced row
            for (int i = N-1; i >= 0; i--)
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

            return C;
        }
    
        
};

#endif /* matrixmath_h */
