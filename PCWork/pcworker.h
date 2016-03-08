#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QElapsedTimer>
#include "project_common.h"
#include "IO/glvertexmanager.h"
#include "KernelTest/descriptor.h"
#include "ClWork/clworker.h"
#include "ClWork/cloperators.h"
#include "Share/sharedenums.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void SetInputs(QImage& srcColorImg, cl_float4* srcPointCloud, int inViewOption);
    void Work();

private:
    void DrawPointCloud(cl_float4* pointCloud, cl_float4* normalCloud, int viewOption);
    void CheckNaN(cl_float4* points);

    cl_float4*      pointCloud;
    cl_float4*      normalCloud;
    cl_int*         neighborIndices;
    cl_int*         numNeighbors;
    DescType*       descriptorCloud;
    QImage          colorImg;
    int             viewOption;

    CLWorker*       clworker;
    QElapsedTimer   eltimer;
};

#endif // PCWORKER_H
