#ifndef LUDECOMP_H
#define LUDECOMP_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>
#include <QElapsedTimer>
#include <QDebug>
using namespace std;
typedef float  dtype;
#define DIM     5
#define SQDIM   (DIM*DIM)


class LUDecomp
{
public:
    LUDecomp();

    void CompareMethods();
    int Doolittle_LU_Decomposition_with_Pivoting(dtype *A, int pivot[], int n);
    int Doolittle_LU_with_Pivoting_Solve(dtype *A, dtype B[], int pivot[], dtype x[], int n);
//    void gauss(dtype* A, dtype* x, dtype* b, int n);
    vector<dtype> gauss(vector< vector<dtype> > A);
    void GaussElim(int D, dtype* A, dtype* x);

    void CreateRandLinearEq(int d, dtype* A, dtype* b, int offset=1);
    void CreateRandLinearEq(int d, vector< vector<dtype> >& A, int offset=1);
    void CreateRandLinearEq(int d, dtype* A, int offset=1);
    float ComputeError(int d, dtype* A, dtype* x, dtype* b);
    float ComputeError(vector< vector<dtype> > A, vector<dtype> x);
    float ComputeError(int d, dtype* A, dtype* x);

    // These functions print matrices and vectors in a nice format
    void coutMatrix(int d, dtype*m)
    {
       cout<<'\n';
       for(int i=0;i<d;++i){
          for(int j=0;j<d;++j)cout<<setw(14)<<m[i*d+j];
          cout<<'\n';
       }
    }

    void coutVector(int d, dtype* v)
    {
       cout<<'\n';
       for(int j=0;j<d;++j)cout<<setw(14)<<v[j];
       cout<<'\n';
    }

    void coutVector(int d, int* v)
    {
       cout<<'\n';
       for(int j=0;j<d;++j)cout<<setw(14)<<v[j];
       cout<<'\n';
    }

    // Crout uses unit diagonals for the upper triangle
    void Crout(int d, dtype*S, dtype*D)
    {
       for(int k=0;k<d;++k){
          for(int i=k;i<d;++i){
             dtype sum=0.;
             for(int p=0;p<k;++p)sum+=D[i*d+p]*D[p*d+k];
             D[i*d+k]=S[i*d+k]-sum; // not dividing by diagonals
          }
          for(int j=k+1;j<d;++j){
             dtype sum=0.;
             for(int p=0;p<k;++p)sum+=D[k*d+p]*D[p*d+j];
             D[k*d+j]=(S[k*d+j]-sum)/D[k*d+k];
          }
       }
    }
    void solveCrout(int d, dtype*LU, dtype*b, dtype*x)
    {
       dtype y[d];
       for(int i=0;i<d;++i){
          dtype sum=0.;
          for(int k=0;k<i;++k)sum+=LU[i*d+k]*y[k];
          y[i]=(b[i]-sum)/LU[i*d+i];
       }
       for(int i=d-1;i>=0;--i){
          dtype sum=0.;
          for(int k=i+1;k<d;++k)sum+=LU[i*d+k]*x[k];
          x[i]=(y[i]-sum); // not dividing by diagonals
       }
    }


    // BEST in speed and memory
    // Doolittle uses unit diagonals for the lower triangle
    void Doolittle(int d, dtype*S, dtype*D)
    {
       for(int k=0;k<d;++k){
          for(int j=k;j<d;++j){
             dtype sum=0.;
             for(int p=0;p<k;++p)sum+=D[k*d+p]*D[p*d+j];
             D[k*d+j]=(S[k*d+j]-sum); // not dividing by diagonals
          }
          for(int i=k+1;i<d;++i){
             dtype sum=0.;
             for(int p=0;p<k;++p)sum+=D[i*d+p]*D[p*d+k];
             D[i*d+k]=(S[i*d+k]-sum)/D[k*d+k];
          }
       }
    }
    void solveDoolittle(int d, dtype*LU, dtype*b, dtype*x)
    {
       dtype y[d];
       for(int i=0;i<d;++i){
          dtype sum=0.;
          for(int k=0;k<i;++k)sum+=LU[i*d+k]*y[k];
          y[i]=(b[i]-sum); // not dividing by diagonals
       }
       for(int i=d-1;i>=0;--i){
          dtype sum=0.;
          for(int k=i+1;k<d;++k)sum+=LU[i*d+k]*x[k];
          x[i]=(y[i]-sum)/LU[i*d+i];
       }
    }

    int iterIndex = 0;
};

#endif // LUDECOMP_H
