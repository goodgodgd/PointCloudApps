#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QColor>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "IO/glvertexmanager.h"
#include "KernelTest/descriptor.h"
#include "shapedescriptor.h"
#include "ClWork/clworker.h"
#include "ClWork/cloperators.h"
#include "Share/sharedenums.h"
#include "PCWork/planeextractor.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void SetInputs(QImage& srcColorImg, cl_float4* srcPointCloud, int inViewOption);
    void Work();
    void DrawPointCloud(int viewOption);

private:
    inline cl_float4 ConvertDescriptorToColor(cl_float4 descriptor);

    CLWorker*       clworker;
    ShapeDescriptor shapeDesc;
    QElapsedTimer   eltimer;

    QColor*         qcolor;
    PlaneExtractor* planeextractor;
    cl_float4*      pointCloud;
    cl_float4*      normalCloud;
    cl_int*         neighborIndices;
    cl_int*         numNeighbors;
    DescType*       descriptorCloud;
    QImage          colorImg;
    int             viewOption;

};

#endif // PCWORKER_H
