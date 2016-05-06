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
#include "IO/imageconverter.h"
#include "segment.h"
#include "Test/testinobjcluster.h"

struct ImLine
{
    cl_int2 endPixels[2];
    float a, b, c;  // ax+by=c

    inline bool IsXAxisMajor() const
    { return (fabsf(a) < fabsf(b)); }

    inline int GetY(const int X) const
    { return (int)((-a*X+c)/b); }

    inline int GetX(const int Y) const
    { return (int)((-b*Y+c)/a); }

    inline void OrthogonalTo(const ImLine& srcLine, const cl_int2 pixel)
    {
        this->a = srcLine.b;
        this->b = -srcLine.a;
        this->c = this->a*pixel.x + this->b*pixel.y;
    }
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
    vecPixels virutalPixels;

private:
    void InitClustering(SharedData* shdDat);

    void AbsorbTinyPlanes();
    bool CanAbsorbSmallPlane(const Segment& largerPlane, const Segment& smallerPlane);
    inline bool DoRectsOverlap(const ImRect& leftRect, const ImRect& rightRect);
    inline ImRect OverlappingRect(const ImRect& leftRect, const ImRect& rightRect);
    bool ArePlanesConnected(const ImRect& ovlRect, const Segment& firstPlane, const Segment& secondPlane, vecPairOfPixels& connPixels);
    bool ArePixelsConnected(const int leftIdx, const cl_float4& leftNormal, const int rightIdx, const cl_float4& rightNormal);
    bool IncludeTinyPlane(const Segment& largerPlane, const Segment& smallerPlane);
    void AbsorbPlane(Segment& largerPlane, Segment& smallerPlane);

    void MergeLargePlanes();
    void MergePlanePairs(vecSegment& planes, vecPairOfInts& mergeList);
    void UpdateIndexMap(mapPairOfInts& indexMap, const int fromValue, const int becomeValue);

    bool ArePlanesInTheSameObject(const Segment& firstPlane, const Segment& secondPlane);
    float InnerAngleBetweenPlanes(const Segment& firstPlane, const Segment& secondPlane, const vecPairOfPixels& connPixels);
    ImLine FitLine2D(const vecPairOfPixels& connPixels);
    bool IsFirstUpperPlane(const ImLine& border, const vecPairOfPixels& connPixels);
    cl_int2 PickBorderCenter(const ImLine& border, const int firstID, const int secondID);
    inline cl_int2 SearchValidPixelOnLine(const ImLine& border, const cl_int2& centerPixel, const int firstID, const int secondID);
    Segment ScalePlaneToIncludePoint(const Segment& srcPlane, const cl_float4& point);
    cl_float4 VirtualPointOnPlaneAroundBorder(const Segment& plane, const ImLine& border, const cl_int2& borderCenter, bool upperPlane);
    cl_float4 PlaneDirectionFromBorder(const cl_float4& thisPlaneNormal, const cl_float4& borderDirection, const cl_float4& roughDirection);
    float AngleBetweenVectorsDegree(const cl_float4& v1, const cl_float4& v2);
    bool DetermineConvexityByHeight(const Segment& firstPlane, const Segment& secondPlane);

    void ExtractValidSegments(const vecSegment& planes, vecSegment& objects);

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
