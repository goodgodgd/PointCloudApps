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
#include "PCWork/planeextractor.h"
#include "Share/sharedenums.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void SetInputs(QImage& srcColorImg, cl_float4* srcPointCloud, int inViewOption);
    void Work();
    void DrawPointCloud(int viewOption);

private:
    friend class planeextractor;
    inline cl_float4 ConvertDescriptorToColor(cl_float4 descriptor);
    void CheckNaN(cl_float4* points);

    CLWorker*       clworker;
    QColor*         qcolor;
    ShapeDescriptor shapeDesc;
    QElapsedTimer   eltimer;

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
