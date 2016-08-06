#include "linearsolver.h"

void CreateRandLinearEq(const int d, dtype* Ab, int offset)
{
    for(int i=0; i<d; i++)
        for(int k=0; k<(d+1); k++)
            Ab[i*(d+1)+k] = (i*k+i+k)%(i+k+1);

//    for(int i=0; i<d*(d+1); i++)
//        Ab[i] = (dtype)(rand()%10+offset)/(dtype)(rand()%10+1);
}

void SolveLinearEq(const int dim, dtype* Ab_io, dtype* x_out)
{
    int width = dim+1;

    for(int i=0; i<dim-1; i++)
    {
        // Search for maximum in this column
        dtype maxEl = fabsf(Ab_io[i*width+i]);
        int maxRow = i;
        for (int k=i+1; k<dim; k++)
        {
            if (fabsf(Ab_io[k*width+i]) > maxEl)
            {
                maxEl = fabsf(Ab_io[k*width+i]);
                maxRow = k;
            }
        }

        // Swap maximum row with current row (column by column)
        for (int k=i; k<width;k++)
        {
            dtype tmp = Ab_io[maxRow*width+k];
            Ab_io[maxRow*width+k] = Ab_io[i*width+k];
            Ab_io[i*width+k] = tmp;
        }

        // Make all rows below this one 0 in current column
        for (int k=i+1; k<dim; k++)
        {
            dtype c = -Ab_io[k*width+i]/Ab_io[i*width+i];
            for (int j=i; j<width; j++)
            {
                if (i==j)
                    Ab_io[k*width+j] = 0;
                else
                    Ab_io[k*width+j] += c * Ab_io[i*width+j];
            }
        }
    }

    // Solve equation Ax=b for an upper triangular matrix Ab_io
    for (int i=dim-1; i>=0; i--)
    {
        x_out[i] = Ab_io[i*width+dim]/Ab_io[i*width+i];
        for (int k=i-1;k>=0; k--)
            Ab_io[k*width+dim] -= Ab_io[k*width+i] * x_out[i];
    }
}

dtype ComputeError(const int d, dtype* Ab, dtype* x)
{
    dtype dot, err = 0.f;
    for(int r=0;r<d;++r)
    {
        dot = 0.f;
        for(int c=0;c<d;++c)
            dot += Ab[r*(d+1)+c]*x[c];
        err += fabsf(Ab[r*(d+1)+d] - dot);
    }
    return err;
}

void PrintMatrix(const int rows, const int cols, dtype* M, const char* title)
{
    cout << title << endl;
    for(int r=0;r<rows;++r)
    {
        for(int c=0;c<cols;++c)
            cout << setprecision(5) << setw(13) << M[r*cols+c];
        cout << endl;
    }
}

void PrintVector(const int d, dtype* V, const char* title)
{
    cout << title << " ";
    for(int r=0;r<d;++r)
        cout << setw(13) << V[r];
    cout << endl;
}
