#include "ludecomp.h"

LUDecomp::LUDecomp()
{
}

void LUDecomp::CompareMethods()
{
    float A[SQDIM];
    float LU[SQDIM];
    float b[DIM];
    float b_copy[DIM];
    float x[DIM];
    int pivot[DIM];
    srand(0);
    int iternum = 1000;
    QElapsedTimer eltimer;
    float error;

    srand(0);
    error = 0;
    eltimer.start();
    for(int i=0; i<iternum; i++)
    {
        iterIndex = i;
        CreateRandLinearEq(DIM, A, b, 1);
        Crout(DIM,A,LU);
        solveCrout(DIM,LU,b,x);
        error += ComputeError(DIM,A,x,b);
    }
    qDebug() << "Crout took" << eltimer.nsecsElapsed()/1000 << "us" << "error" << error;

    srand(0);
    error = 0;
    eltimer.start();
    for(int i=0; i<iternum; i++)
    {
        iterIndex = i;
        CreateRandLinearEq(DIM, A, b, 1);
        Doolittle(DIM,A,LU);
        solveDoolittle(DIM,LU,b,x);
        error += ComputeError(DIM,A,x,b);
    }
    qDebug() << "Doolittle took" << eltimer.nsecsElapsed()/1000 << "us" << "error" << error;

    srand(0);
    error = 0;
    eltimer.start();
    for(int i=0; i<iternum; i++)
    {
        iterIndex = i;
        CreateRandLinearEq(DIM, A, b);
        memcpy(LU, A, SQDIM*sizeof(dtype));
        memcpy(b_copy, b, DIM*sizeof(dtype));
        Doolittle_LU_Decomposition_with_Pivoting(LU, pivot, DIM);
        Doolittle_LU_with_Pivoting_Solve(LU, b, pivot, x, DIM);
        solveDoolittle(DIM,LU,b,x);
        error += ComputeError(DIM,A,x,b_copy);
    }
    qDebug() << "Doolittle_pivot took" << eltimer.nsecsElapsed()/1000 << "us" << "error" << error;

    srand(0);
    error = 0;
    eltimer.start();
    vector< vector<dtype> > vA;
    vector<dtype> vx;
    for(int i=0; i<iternum; i++)
    {
        iterIndex = i;
        CreateRandLinearEq(DIM, vA, 0);
        vx = gauss(vA);
        error += fabs(ComputeError(vA,vx));
    }
    qDebug() << "Gauss took" << eltimer.nsecsElapsed()/1000 << "us" << "error" << error;


    srand(0);
    error = 0;
    eltimer.start();
    dtype Ab[DIM*(DIM+1)];
    dtype Ab_bk[DIM*(DIM+1)];
    for(int i=0; i<iternum; i++)
    {
        iterIndex = i;
        CreateRandLinearEq(DIM, Ab, 0);
        memcpy(Ab_bk, Ab, sizeof(Ab));
        GaussElim(DIM, Ab, x);
        error += ComputeError(DIM, Ab_bk, x);
    }
    qDebug() << "GaussElim took" << eltimer.nsecsElapsed()/1000 << "us" << "error" << error;
}

void LUDecomp::CreateRandLinearEq(int d, dtype* A, dtype* b, int offset)
{
    for(int i=0; i<d*d; i++)
        A[i] = (float)(rand()%10+offset)/(float)(rand()%10+1);
    for(int i=0; i<d; i++)
        b[i] = (float)(rand()%10)/(float)(rand()%10+1);
}

void LUDecomp::CreateRandLinearEq(int d, vector<vector<dtype>>& A, int offset)
{
    A.resize(d);
    for(int r=0; r<d; r++)
    {
        A[r].resize(d+1);
        for(int c=0; c<d+1; c++)
            A[r][c] = (float)(rand()%10+offset)/(float)(rand()%10+1);
    }
}

void LUDecomp::CreateRandLinearEq(int d, dtype* A, int offset)
{
    for(int i=0; i<d*(d+1); i++)
        A[i] = (float)(rand()%10+offset)/(float)(rand()%10+1);
}


vector<dtype> LUDecomp::gauss(vector< vector<dtype> > A)
{
    int n = A.size();
    vector<vector<dtype>> A_bk = A;

    for (int i=0; i<n; i++) {
        // Search for maximum in this column
        dtype maxEl = fabsf(A[i][i]);
        int maxRow = i;
        for (int k=i+1; k<n; k++) {
            if (fabsf(A[k][i]) > maxEl) {
                maxEl = fabsf(A[k][i]);
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        for (int k=i; k<n+1;k++) {
            dtype tmp = A[maxRow][k];
            A[maxRow][k] = A[i][k];
            A[i][k] = tmp;
        }

        // Make all rows below this one 0 in current column
        for (int k=i+1; k<n; k++) {
            dtype c = -A[k][i]/A[i][i];
            for (int j=i; j<n+1; j++) {
                if (i==j) {
                    A[k][j] = 0;
                } else {
                    A[k][j] += c * A[i][j];
                }
            }
        }
    }

    // Solve equation Ax=b for an upper triangular matrix A
    vector<dtype> x(n);
    for (int i=n-1; i>=0; i--) {
        x[i] = A[i][n]/A[i][i];
        for (int k=i-1;k>=0; k--) {
            A[k][n] -= A[k][i] * x[i];
        }
    }
/*
    if(iterIndex==1)
    {
        cout<< "gauss2" << endl;
        for(int i=0;i<n;++i)
        {
            for(int j=0;j<n+1;++j)
                cout<<setw(14)<<A[i][j];
            cout<<'\n';
        }
        for(int i=0;i<n;++i)
            cout<<setw(14)<<x[i];
        cout << '\n';
        cout << "error " << ComputeError(A_bk, x) << endl;
    }
*/
    return x;
}

void LUDecomp::GaussElim(int D, dtype* A, dtype* x)
{
    int W = D+1;

    for(int i=0; i<D; i++)
    {
        // Search for maximum in this column
        dtype maxEl = fabsf(A[i*W+i]);
        int maxRow = i;
        for (int k=i+1; k<D; k++)
        {
            if (fabsf(A[k*W+i]) > maxEl)
            {
                maxEl = fabsf(A[k*W+i]);
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        for (int k=i; k<W;k++)
        {
            dtype tmp = A[maxRow*W+k];
            A[maxRow*W+k] = A[i*W+k];
            A[i*W+k] = tmp;
        }

        // Make all rows below this one 0 in current column
        for (int k=i+1; k<D; k++)
        {
            dtype c = -A[k*W+i]/A[i*W+i];
            for (int j=i; j<W; j++)
            {
                if (i==j)
                    A[k*W+j] = 0;
                else
                    A[k*W+j] += c * A[i*W+j];
            }
        }
    }

    // Solve equation Ax=b for an upper triangular matrix A
    for (int i=D-1; i>=0; i--)
    {
        x[i] = A[i*W+D]/A[i*W+i];
        for (int k=i-1;k>=0; k--)
            A[k*W+D] -= A[k*W+i] * x[i];
    }
/*
    if(iterIndex==1)
    {
        cout<< "GaussElim" << endl;
        for(int i=0;i<D;++i)
        {
            for(int j=0;j<W;++j)
                cout<<setw(14)<<A[i*W+j];
            cout<<'\n';
        }
        coutVector(D, x);
        cout << "error " << ComputeError(D, A, x) << endl;
    }
*/
}

float LUDecomp::ComputeError(vector< vector<dtype> > A, vector<dtype> x)
{
    int n = A.size();
    float dot, err = 0.f;
    for(int r=0;r<n;++r)
    {
        dot = 0.f;
        for(int c=0;c<n;++c)
            dot += A[r][c]*x[c];
        err += fabsf(A[r][n] - dot);
    }
    return err;
}

float LUDecomp::ComputeError(int d, dtype* A, dtype* x)
{
    float dot, err = 0.f;
    for(int r=0;r<d;++r)
    {
        dot = 0.f;
        for(int c=0;c<d;++c)
            dot += A[r*(d+1)+c]*x[c];
        err += fabsf(A[r*(d+1)+d] - dot);
    }
    return err;
}

float LUDecomp::ComputeError(int d, dtype* A, dtype* x, dtype* b)
{
    float dot, err = 0.f;
    for(int r=0;r<d;++r)
    {
        dot = 0.f;
        for(int c=0;c<d;++c)
            dot += A[r*d+c]*x[c];
        err += fabsf(b[r] - dot);
    }
    return err;
}

int LUDecomp::Doolittle_LU_Decomposition_with_Pivoting(dtype *A, int pivot[], int n)
{
   int i, j, k, p;
   dtype *p_k, *p_row, *p_col;
   dtype max;


//         For each row and column, k = 0, ..., n-1,

   for (k = 0, p_k = A; k < n; p_k += n, k++) {

//            find the pivot row

      pivot[k] = k;
      max = fabs( *(p_k + k) );
      for (j = k + 1, p_row = p_k + n; j < n; j++, p_row += n) {
         if ( max < fabs(*(p_row + k)) ) {
            max = fabs(*(p_row + k));
            pivot[k] = j;
            p_col = p_row;
         }
      }

//     and if the pivot row differs from the current row, then
//     interchange the two rows.

      if (pivot[k] != k)
         for (j = 0; j < n; j++) {
            max = *(p_k + j);
            *(p_k + j) = *(p_col + j);
            *(p_col + j) = max;
         }

//                and if the matrix is singular, return error


      if ( *(p_k + k) == 0.0 ) return -1;

//      otherwise find the lower triangular matrix elements for column k.

      for (i = k+1, p_row = p_k + n; i < n; p_row += n, i++) {
         *(p_row + k) /= *(p_k + k);
      }

//            update remaining matrix

      for (i = k+1, p_row = p_k + n; i < n; p_row += n, i++)
         for (j = k+1; j < n; j++)
            *(p_row + j) -= *(p_row + k) * *(p_k + j);

   }

   return 0;
}

int LUDecomp::Doolittle_LU_with_Pivoting_Solve(dtype *A, dtype B[], int pivot[],
                                                              dtype x[], int n)
{
   int i, k;
   dtype *p_k;
   dtype dum;

//         Solve the linear equation Lx = B for x, where L is a lower
//         triangular matrix with an implied 1 along the diagonal.

   for (k = 0, p_k = A; k < n; p_k += n, k++) {
      if (pivot[k] != k) {dum = B[k]; B[k] = B[pivot[k]]; B[pivot[k]] = dum; }
      x[k] = B[k];
      for (i = 0; i < k; i++) x[k] -= x[i] * *(p_k + i);
   }

//         Solve the linear equation Ux = y, where y is the solution
//         obtained above of Lx = B and U is an upper triangular matrix.

   for (k = n-1, p_k = A + n*(n-1); k >= 0; k--, p_k -= n) {
      if (pivot[k] != k) {dum = B[k]; B[k] = B[pivot[k]]; B[pivot[k]] = dum; }
      for (i = k + 1; i < n; i++) x[k] -= x[i] * *(p_k + i);
      if (*(p_k + k) == 0.0) return -1;
      x[k] /= *(p_k + k);
   }

   return 0;
}
