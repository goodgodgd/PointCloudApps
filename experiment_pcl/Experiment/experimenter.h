#ifndef EXPERIMENTER_H
#define EXPERIMENTER_H

#include <QImage>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/pose6dof.h"
#include "IO/drawutils.h"
#include "IO/imageconverter.h"
#include "ClUtils/cloperators.h"
#include "PCWork/radiussearch.h"
#include "PCWork/normalmaker.h"
#include "PCWork/Descriptor/descriptormaker.h"
#include "PCWork/Descriptor/descriptormakerbycpu.h"
#include "PCWork/Clustering/clusterer.h"
#include "PCWork/Clustering/planeclusterpolicy.h"
#include "PCWork/Clustering/smallplanemerger.h"
#include "convertertopcl.h"
#include "pointtracker.h"
#include "pointsampler.h"
#include "pcldescriptors.h"
#include "IOExpm/trackrecorder.h"
#include "IOExpm/objectrecorder.h"

class Experimenter
{
public:
    Experimenter();
    ~Experimenter();
    void Work(SharedData* shdDat, bool bObject=false, bool bWrite=false);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel);
    void CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud);

private:
    void SearchNeighborsAndCreateNormal(SharedData* shdDat);
    void ComputeDescriptorsCpu(SharedData* shdDat);
    void ComputeDescriptorsGpu(SharedData* shdDat);
    void CheckDataValidity(SharedData* shdDat, const cl_float4* descriptorsGpu, const AxesType* prinAxesGpu);
    void FindPlanes(SharedData* shdDat);
    void SetPlanesNull(SharedData* shdDat);
    void CheckObjectValidity(SharedData* shdDat, const float minSize);
    void SetBasicNullity(SharedData* shdDat);
    void SetDescriptorNullity(SharedData* shdDat);

    QElapsedTimer eltimer;
    cl_int* neighborIndices;
    cl_int* numNeighbors;
    ArrayData<cl_uchar> nullData;

    RadiusSearch neibSearcher;
    NormalMaker normalMaker;
    Clusterer<PlaneClusterPolicy> planeClusterer;
    SmallPlaneMerger    planeMerger;

    DescriptorMaker descriptorMaker;
    DescriptorMakerByCpu descriptorMakerCpu;
    PclDescriptors pclDescs;

    PointTracker pointTracker;
    PointSampler pointSampler;
    TrackRecorder trackRecorder;
    ObjectRecorder objectRecorder;

    friend class PCAppsExperiment;
};

#endif // EXPERIMENTER_H
