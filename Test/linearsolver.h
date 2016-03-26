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

extern int iter;
void TestSolver();
void CreateRandLinearEq(const int d, dtype* Ab, int offset);
// dim: # equations
// Ab_io: input and output variable, represents linear equation, Ax = b, Ab = [A|b]
// x: output variable, represents solution x
void SolveLinearEq(const int dim, dtype* Ab_io, dtype* x_out);
void PrintMatrix(const int rows, const int cols, dtype* M, const char* title=NULL);
void PrintVector(const int d, dtype* V, const char* title=NULL);
dtype ComputeError(const int d, dtype* A, dtype* x);

#endif // LUDECOMP_H
