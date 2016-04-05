#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/sharedenums.h"
#include "Share/drawutils.h"
#include "IO/glvertexmanager.h"
#include "radiussearch.h"
#include "normalmaker.h"
#include "descriptormaker.h"
#include "ClUtils/cloperators.h"
#include "Test/Proto/normalsmoother.h"
#include "Test/Proto/pointsmoother.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void Work(QImage& srcColorImg, cl_float4* srcPointCloud);
    void DrawPointCloud(int viewOption);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void MarkPoint3D(QPoint pixel, int viewOption);
    void DrawOnlyNeighbors(QPoint pixel, int viewOption);

private:
    RadiusSearch    neibSearcher;
    NormalMaker     normalMaker;
    DescriptorMaker descriptorMaker;
    NormalSmoother  normalSmoother;
    PointSmoother   pointSmoother;

    QElapsedTimer   eltimer;

    cl_float4*      pointCloud;
    cl_float4*      normalCloud;
    cl_int*         neighborIndices;
    cl_int*         numNeighbors;
    DescType*       descriptorCloud;
    QImage          colorImg;
};

#endif // PCWORKER_H
