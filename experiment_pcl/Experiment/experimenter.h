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
#include "PCWork/descriptormaker.h"
#include "PCWork/descriptormakerbycpu.h"
#include "convertertopcl.h"
#include "pointtracker.h"
#include "pcldescriptors.h"
#include "trackrecorder.h"
#include "objectrecorder.h"

class Experimenter
{
public:
    Experimenter();
    ~Experimenter();
    void Work(const QImage& srcColorImg, const QImage& srcDepthImg, const Pose6dof& srcPose, SharedData* shdDat, bool bObject=false);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel);
    void CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud);

private:
    float CheckObjectSize(SharedData* shdDat);
    void CreateNormalAndNullity(SharedData* shdDat);
    cl_uchar* CreateNullityMap(SharedData* shdDat);
    void ComputeCWGDescriptor(SharedData* shdDat);

    QElapsedTimer eltimer;
    RadiusSearch neibSearcher;
    NormalMaker normalMaker;
    DescriptorMaker cwgMaker_gpu;
    DescriptorMakerByCpu cwgMaker_cpu;
    PclDescriptors pclDescs;

    cl_int* neighborIndices;
    cl_int* numNeighbors;
    QImage colorImg;

    PointTracker pointTracker;
    TrackRecorder trackRecorder;
    ObjectRecorder objectRecorder;

    friend class PCAppsExperiment;
};

#endif // EXPERIMENTER_H
