#include "linearsolver.h"

int iter = 0;

void TestSolver()
{
    const int DIM = 9;
    dtype* Ab = new dtype[DIM*(DIM+1)];
    dtype* Ab_bk = new dtype[DIM*(DIM+1)];
    dtype* x = new dtype[DIM];
    int iternum = 1000;
    QElapsedTimer eltimer;
    dtype error;

    srand(0);
    error = 0;
    eltimer.start();
    for(int i=0; i<iternum; i++)
    {
        iter = i;
        CreateRandLinearEq(DIM, Ab, 1);
        memcpy(Ab_bk, Ab, sizeof(dtype)*DIM*(DIM+1));
        SolveLinearEq(DIM, Ab, x);
        dtype tmperr = ComputeError(DIM, Ab_bk, x);
        error += tmperr;

        if(tmperr>0.01f)
        {
            cout << "Big error " << tmperr << endl;
            PrintMatrix(DIM, DIM+1, Ab_bk);
            PrintMatrix(DIM, DIM+1, Ab);
            PrintVector(DIM, x);
        }
    }
    qDebug() << "SolveLinearEq took" << eltimer.nsecsElapsed()/1000 << "us" << "error" << error;
    delete[] Ab;
    delete[] Ab_bk;
    delete[] x;
}

void CreateRandLinearEq(const int d, dtype* Ab, int offset)
{
    for(int i=0; i<d*(d+1); i++)
        Ab[i] = (dtype)(rand()%10+offset)/(dtype)(rand()%10+1);
}

void SolveLinearEq(const int dim, dtype* Ab_io, dtype* x_out)
{
    int width = dim+1;

    for(int i=0; i<dim; i++)
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

//        PrintMatrix(dim, width, Ab_io, "\nprocess");
    }

    // Solve equation Ax=b for an upper triangular matrix Ab_io
    for (int i=dim-1; i>=0; i--)
    {
        x_out[i] = Ab_io[i*width+dim]/Ab_io[i*width+i];
        for (int k=i-1;k>=0; k--)
            Ab_io[k*width+dim] -= Ab_io[k*width+i] * x_out[i];
    }

//    if(iter==1)
//    {
//        cout << "Solve linear" << endl;
//        PrintMatrix(dim, Ab_io);
//        PrintVector(dim, x_out);
//    }
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
