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
#include "Clustering/objectclusterer.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void Work(const QImage& srcColorImg, const QImage& srcDepthImg, SharedData* shdDat);
    cl_uchar* CreateNullityMap(const cl_float4* pointCloud, const cl_float4* normalCloud, const cl_float4* descriptors);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel);
    void CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud, const cl_float4* descriptors);

private:
    RadiusSearch    neibSearcher;
    NormalMaker     normalMaker;
    DescriptorMaker descriptorMaker;
    NormalSmoother  normalSmoother;
    PointSmoother   pointSmoother;
    Clusterer<PlaneClusterPolicy> planeClusterer;
    ObjectClusterer objectCluster;

    QElapsedTimer   eltimer;

    cl_int*         neighborIndices;
    cl_int*         numNeighbors;
//    cl_float4*      pointCloud;
//    cl_float4*      normalCloud;
//    DescType*       descriptors;
    QImage          colorImg;

    friend class MainWindow;
};

#endif // PCWORKER_H
