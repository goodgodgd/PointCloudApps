#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/sharedenums.h"
#include "Share/shareddata.h"
#include "IO/drawutils.h"
#include "IO/glvertexmanager.h"
#include "IO/imageconverter.h"
#include "radiussearch.h"
#include "normalmaker.h"
#include "descriptormaker.h"
#include "ClUtils/cloperators.h"
#include "Test/Proto/normalsmoother.h"
#include "Test/Proto/pointsmoother.h"
#include "Test/testnormalvalidity.h"
#include "Clustering/clusterer.h"
#include "Clustering/planeclusterpolicy.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void Work(SharedData* shdDat, const QImage& colorImg, const QImage& depthImg);
    void DrawPointCloud(int viewOption);
    cl_uchar* CreateNullityMap();
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void MarkPoint3D(QPoint pixel);
    void DrawOnlyNeighbors(QPoint pixel, int viewOption);
    void CheckDataValidity();

private:
    RadiusSearch    neibSearcher;
    NormalMaker     normalMaker;
    DescriptorMaker descriptorMaker;
    NormalSmoother  normalSmoother;
    PointSmoother   pointSmoother;
    Clusterer<PlaneClusterPolicy> planeClusterer;

    QElapsedTimer   eltimer;

    cl_float4*      pointCloud;
    cl_float4*      normalCloud;
    cl_int*         neighborIndices;
    cl_int*         numNeighbors;
    DescType*       descriptors;
    cl_uchar*       nullityMap;
    cl_int*         planeMap;
    QImage          colorImg;

    friend class MainWindow;
};

#endif // PCWORKER_H
