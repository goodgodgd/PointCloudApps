#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
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
#include "Clustering/smallplanemerger.h"
#include "Clustering/objectclusterer.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void Work(const QImage& srcColorImg, const QImage& srcDepthImg, SharedData* shdDat);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel);
    void CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud, const cl_float4* descriptors);

private:
    void CreateNormalAndDescriptor(SharedData* shdDat);
    cl_uchar* CreateNullityMap(SharedData* shdDat);

    RadiusSearch    neibSearcher;
    NormalMaker     normalMaker;
    DescriptorMaker descriptorMaker;
    NormalSmoother  normalSmoother;
    PointSmoother   pointSmoother;
    Clusterer<PlaneClusterPolicy> planeClusterer;
    SmallPlaneMerger planeMerger;
    ObjectClusterer objectClusterer;

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
