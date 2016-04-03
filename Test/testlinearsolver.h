#ifndef TESTLINEARSOLVER_H
#define TESTLINEARSOLVER_H

#include "Proto/linearsolver.h"

namespace Test
{

inline void testSolveLinearEq()
{
    const int DIM = 9;
    dtype* Ab = new dtype[DIM*(DIM+1)];
    dtype* Ab_bk = new dtype[DIM*(DIM+1)];
    dtype* x = new dtype[DIM];
    int iternum = 1;
    QElapsedTimer eltimer;
    dtype error;

    srand(0);
    error = 0;
    eltimer.start();
    for(int i=0; i<iternum; i++)
    {
        CreateRandLinearEq(DIM, Ab, 1);
        memcpy(Ab_bk, Ab, sizeof(dtype)*DIM*(DIM+1));
        SolveLinearEq(DIM, Ab, x);
        dtype tmperr = ComputeError(DIM, Ab_bk, x);
        error += tmperr;

        PrintMatrix(DIM, DIM+1, Ab_bk, "original");
        PrintMatrix(DIM, DIM+1, Ab, "result");
        PrintVector(DIM, x, "solution");
    }
    qDebug() << "SolveLinearEq took" << eltimer.nsecsElapsed()/1000 << "us" << "error" << error;
    delete[] Ab;
    delete[] Ab_bk;
    delete[] x;
}

}
#endif // TESTLINEARSOLVER_H
