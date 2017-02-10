#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/pose6dof.h"
#include "IO/drawutils.h"
#include "IO/glvertexmanager.h"
#include "ClUtils/cloperators.h"
#include "radiussearch.h"
#include "normalmaker.h"
#include "Descriptor/descriptormaker.h"
#include "Descriptor/descriptormakerbycpu.h"
#include "Clustering/clusterer.h"
#include "Clustering/planeclusterpolicy.h"
#include "Clustering/smallplanemerger.h"
#include "Clustering/objectclusterer.h"
#include "Clustering/clustererbydbrect.h"

//#define COMPARE_DESC_CPU

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void Work(const QImage& srcColorImg, const QImage& srcDepthImg, const Pose6dof& framePose, SharedData* shdDat);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel);

private:
    void SearchNeighborsAndCreateNormal(SharedData* shdDat);
    void ComputeDescriptorsCpu(SharedData* shdDat);
    void ComputeDescriptorsGpu(SharedData* shdDat);
    void ClusterPointsOfObjects(SharedData* shdDat);
    void SetBasicNullity(SharedData* shdDat);
    void SetDescriptorNullity(SharedData* shdDat);
    void CheckDataValidity(SharedData* shdDat, const cl_float4* descriptorsGpu, const AxesType* prinAxesGpu);

    RadiusSearch        neibSearcher;
    NormalMaker         normalMaker;
    DescriptorMaker     descriptorMaker;
    DescriptorMakerByCpu descriptorMakerCpu;

    Clusterer<PlaneClusterPolicy> planeClusterer;
    SmallPlaneMerger    planeMerger;
    ObjectClusterer     objectClusterer;
    ClustererByDbRect   clustererByDbRect;

    QElapsedTimer       eltimer;

    ArrayData<cl_uchar> nullData;
    cl_int* neighborIndices;
    cl_int* numNeighbors;
//    cl_float4*      pointCloud;
//    cl_float4*      normalCloud;
//    DescType*       descriptors;
    QImage colorImg;

    QFile timingFile;
    friend class MainWindow;
};

#endif // PCWORKER_H
