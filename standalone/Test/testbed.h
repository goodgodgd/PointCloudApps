#ifndef TESTBED_H
#define TESTBED_H

#include "testdescriptor.h"
#include "testindexsort.h"
#include "testlinearsolver.h"
#include "testnoise.h"

inline void DoTest()
{
    Test::TestGaussianRand();
    Test::TestUniformRand();
    return;

    Test::TestDescriptor::ComputeEachDescriptor();
    Test::testBubbleSort();
    Test::testSolveLinearEq();
}

#endif // TESTBED_H
