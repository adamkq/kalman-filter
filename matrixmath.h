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
        static void print(vector<vector <int>> A) //print the matrix.
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
    
        static vector<vector <int>> matAdd(vector<vector <int>> A, vector<vector <int>> B)
        {
            if (A.size() != B.size() || A[0].size() != B[0].size())
            {
                throw invalid_argument(" Matrix dimensions not equal.");
            }
            vector<vector <int>> C;
            for (int i=0; i<A.size(); i++)
            {
                vector <int> row;
                for (int j=0; j<A[0].size(); j++)
                {
                    row.push_back(A[i][j] + B[i][j]);
                }
                C.push_back(row);
            }
            return C;
        }
    
        static vector<vector <int>> matSub(vector<vector <int>> A, vector<vector <int>> B)
            // for semantic clarity, this is a separate function from matAdd
        {
            if (A.size() != B.size() || A[0].size() != B[0].size())
            {
                throw invalid_argument(" Matrix dimensions not equal.");
            }
            vector<vector <int>> C;
            for (int i=0; i<A.size(); i++)
            {
                vector <int> row;
                for (int j=0; j<A[0].size(); j++)
                {
                    row.push_back(A[i][j] - B[i][j]);
                }
                C.push_back(row);
            }
            return C;
        }
    
        static vector<vector <int>> matMult(vector<vector <int>> A, vector<vector <int>> B)
        {
            if (A[0].size() != B.size()) // not square
            {
                throw invalid_argument("Matrix dimensions A and B cannot multiply.");
            }
            vector<vector <int>> C;
            int elem;
            
            for (int i=0; i<A.size(); i++)
            {
                vector <int> row;
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
    
        static vector<vector <int>> TPose(vector<vector <int>> A)
        {
            vector<vector <int>> C(A[0].size(), vector <int> (A.size()));
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
};

#endif /* matrixmath_h */
