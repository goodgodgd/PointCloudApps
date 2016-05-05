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

struct ImLine
{
    cl_int2 endPixels[2];
    float a, b, c;  // ax+by=c
};
typedef std::vector<ImLine> vecLines;

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
    const Segment* GetObjectByID(const int ID);
    vecLines borderLines;
    vecPairOfInts IdPairs;

private:
    void InitClustering(SharedData* shdDat);
    void ExtractValidSegments(const vecSegment& planes, vecSegment& objects);
    bool ArePlanesInTheSameObject(const Segment& largerPlane, const Segment& smallerPlane);
    inline bool DoRectsOverlap(const ImRect& leftRect, const ImRect& rightRect);
    inline ImRect OverlappingRect(const ImRect& leftRect, const ImRect& rightRect);
    bool ArePlanesConnected(const ImRect& ovlRect, const Segment& firstPlane, const Segment& secondPlane, vecPairOfPixels& connPixels);
    bool ArePixelsConnected(const int leftIdx, const cl_float4& leftNormal, const int rightIdx, const cl_float4& rightNormal);
    bool IncludeTinyPlane(const Segment& largerPlane, const Segment& smallerPlane);
    void MergePlanes(Segment& largerPlane, Segment& smallerPlane);
    bool ConvexToEachOther(const Segment& leftPlane, const Segment& rightPlane, const vecPairOfPixels& connPixels);
    ImLine FitLine2D(const vecPairOfPixels& connPixels);
    bool IsFirstUpperPlane(const ImLine& border, const vecPairOfPixels& connPixels);
    cl_float4 PickCenterPointOfBorder(const ImLine& border);
    inline cl_int2 SearchValidPixelOnLine(const ImLine& border, const cl_int2& centerPixel, const int majorAxis, const int minorAxis);
    cl_float4 ComputeVirtualPoints(const Segment& plane, const ImLine& border, bool fisrtPlaneUp);
    float InnerAngleDegree(const cl_float4& borderCenter, const cl_float4& pointOnFirst, const cl_float4& pointOnSecond);
    bool DetermineConvexityByHeight(const Segment& firstPlane, const Segment& secondPlane);


    PairPointNormal ComputePlaneIntersect(const Segment& leftPlane, const Segment& rightPlane);
    PairOfPixels ConvertToImageLine(const PairPointNormal& intersectLine, const ImRect& ovlRect);
    inline cl_float2 ProjectPointOntoImage(const cl_float4& srcpt);

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;
    const cl_int* planeMap;
    vecSegment planes;

    ArrayData<cl_int> objectArray;
    cl_int* objectMap;
    vecSegment objects;
    mapPairOfInts mapIdIndex;
};

#endif // OBJECTCLUSTERER_H
