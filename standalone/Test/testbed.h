#ifndef TESTBED_H
#define TESTBED_H

#include "testdescriptor.h"
#include "testindexsort.h"
#include "testlinearsolver.h"
#include "testnoise.h"
#include "TestReadOBJ.h"

inline void DoTest()
{
    Test::TestReadOBJ();
    return;
    Test::TestGaussianRand();
    Test::TestUniformRand();

    Test::TestDescriptor::ComputeEachDescriptor();
    Test::testBubbleSort();
    Test::testSolveLinearEq();
}

#endif // TESTBED_H
