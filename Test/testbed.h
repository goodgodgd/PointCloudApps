#ifndef TESTBED_H
#define TESTBED_H

#include "testdescriptor.h"
#include "testindexsort.h"
#include "testlinearsolver.h"

inline void DoTest()
{
    return;

    Test::TestDescriptor::ComputeEachDescriptor();
    Test::testBubbleSort();
    Test::testSolveLinearEq();
}

#endif // TESTBED_H
