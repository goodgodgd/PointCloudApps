#ifndef OBJECTCLUSTERER_H
#define OBJECTCLUSTERER_H

#include <utility>
#include <cassert>
#include <QImage>
//#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "Share/project_common.h"
#include "Share/camera_param.h"
#include "Share/shared_data.h"
#include "Share/shared_enums.h"
#include "Share/shared_types.h"
#include "Share/arraydata.h"
#include "ClUtils/cloperators.h"
#include "segment.h"
#include "Test/testinobjcluster.h"

struct Line2D
{
    cl_int2 endPixels[2];
    float a, b, c;  // ax+by=c
};

class ObjectClusterer
{
    enum Enum
    {
        MERGED_PLANE = -1
    };

public:
    ObjectClusterer();

    void ClusterCloudIntoObjects(SharedData* shdDat);
    const cl_int* GetObjectMap();
    const vecSegment* GetObjects();
    vecPairOfPixels imgLines;
    vecPairOfInts planePairs;

private:
    void InitClustering(SharedData* shdDat);
    bool ArePlanesInTheSameObject(const Segment& largerPlane, const Segment& smallerPlane);
    inline bool DoRectsOverlap(const ImRect& leftRect, const ImRect& rightRect);
    inline ImRect OverlappingRect(const ImRect& leftRect, const ImRect& rightRect);
    bool ArePlanesConnected(const ImRect& ovlRect, const Segment& leftPlane, const Segment& rightPlane, vecfPixels& connPixels);
    bool ArePixelsConnected(const int leftIdx, const cl_float4& leftNormal, const int rightIdx, const cl_float4& rightNormal);
    bool IncludeTinyPlane(const Segment& largerPlane, const Segment& smallerPlane);

    bool ConcaveToEachOther(const Segment& leftPlane, const Segment& rightPlane, const vecfPixels& connPixels);
    Line2D FitLine2D(const vecfPixels& connPixels);
    PairPointNormal ComputePlaneIntersect(const Segment& leftPlane, const Segment& rightPlane);
    PairOfPixels ConvertToImageLine(const PairPointNormal& intersectLine, const ImRect& ovlRect);
    inline cl_float2 ProjectPointOntoImage(const cl_float4& srcpt);

    void ClusterPlanes(const vecPairOfInts& pairs, cl_int* objectMap, vecSegment& objects);
    void MergePlanes(Segment& largerPlane, Segment& smallerPlane);

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;
    const cl_int* planeMap;
    vecSegment planes;

    ArrayData<cl_int> objectArray;
    cl_int* objectMap;
    vecSegment objects;
};

#endif // OBJECTCLUSTERER_H
