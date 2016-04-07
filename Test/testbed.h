#ifndef TESTBED_H
#define TESTBED_H

#include "testdescriptor.h"
#include "testindexsort.h"
#include "testlinearsolver.h"
#include "VirtualSensor/virtualdepthsensor.h"

inline void DoTest()
{   
    VirtualDepthSensor depthMaker;
    depthMaker.MakeVirtualDepth("shape.txt", "camera.txt", "noise.txt");
    QImage depthFrame = depthMaker.GetDepthFrame();
    // ImageConverter::ConvertToPointCloud
    // PCWorker::work
    // DrawUtil::DrawPointCloud

    return;

    Test::TestDescriptor::ComputeEachDescriptor();
    Test::testBubbleSort();
    Test::testSolveLinearEq();
}

#endif // TESTBED_H
