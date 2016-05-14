#ifndef OBJECTCLUSTERER_H
#define OBJECTCLUSTERER_H

#include <functional>
#include "objectclusterbase.h"
#include "inclusiontree.h"

class ObjectClusterer : public ObjectClusterBase
{
public:
    ObjectClusterer();

    vecLines borderLines;
    vecPairOfInts IdPairs;
    vecPixels virutalPixels;
    vecFloats betweenAngles;
    vecPairOfPoints pointPairs;
    std::vector<cl_float4> borderPoints;
    vecPairOfFloats heights;

private:
    virtual void MergePlanes();
    void InitDebugData();
    inline void MergeLargePlanes();
    vecOfVecInts InitializedTree(const int size);

    bool ArePlanesInTheSameObject(const Segment& firstPlane, const Segment& secondPlane);
    bool DetermineConvexity(const Segment& firstPlane, const Segment& secondPlane);
    inline bool IsConcave(float angleDegree);

    float InnerAngleBetweenPlanes(const Segment& firstPlane, const Segment& secondPlane, const vecPairOfPixels& connPixels);
    ImLine FitLine2D(const vecPairOfPixels& connPixels);
    bool IsFirstUpperPlane(const ImLine& border, const vecPairOfPixels& connPixels);
    cl_int2 PickBorderCenter(const ImLine& border, const int firstID, const int secondID);
    inline cl_int2 SearchValidPixelOnLine(const ImLine& border, const cl_int2& centerPixel, const int firstID, const int secondID);
    Segment ScalePlaneToIncludePoint(const Segment& srcPlane, const cl_float4& point);
    cl_float4 VirtualPointOnPlaneAroundBorder(const Segment& plane, const ImLine& border, const cl_int2& borderCenter, bool upperPlane);
    cl_float4 PlaneDirectionFromBorder(const cl_float4& thisPlaneNormal, const cl_float4& borderDirection, const cl_float4& roughDirection);
    float AngleBetweenVectorsDegree(const cl_float4& v1, const cl_float4& v2);

    void MergePlanesThroughTree(InclusionTree& includeTree);
    vecInts ExtractMergeList(const vecOfVecInts& includeTree, const int baseIndex);
    void CollectPlanesInSameObject(const Segment& basePlane, const vecOfVecInts& includeTree, const int nodeIndex, vecInts& planeList);
    bool IsIncludable(const int srcIndex, const vecInts& compareList);
};

#endif // OBJECTCLUSTERER_H
