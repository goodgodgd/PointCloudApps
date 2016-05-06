#ifndef OBJECTCLUSTERER_H
#define OBJECTCLUSTERER_H

#include "objectclusterbase.h"

class ObjectClusterer : public ObjectClusterBase
{
public:
    ObjectClusterer();

    vecLines borderLines;
    vecPairOfInts IdPairs;
    vecPixels virutalPixels;
    std::vector<float> betweenAngles;
    vecPairOfPoints pointPairs;
    std::vector<cl_float4> borderPoints;
    vecPairOfFloats heights;

private:
    virtual void MergePlanes();
    void InitDebugData();
    inline void MergeLargePlanes();
    void MergePlanePairs(vecSegment& planes, vecPairOfInts& mergeList);
    inline void UpdateIndexMap(mapPairOfInts& indexMap, const int fromValue, const int becomeValue);
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

    bool DetermineConvexity(const Segment& firstPlane, const Segment& secondPlane, const float angleDegree);
    float HeightFromPlane(const Segment& inputPlane, const Segment& basePlane);
};

#endif // OBJECTCLUSTERER_H
