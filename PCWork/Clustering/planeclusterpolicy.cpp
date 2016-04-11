#include "planeclusterpolicy.h"

PlaneClusterPolicy::PlaneClusterPolicy()
{
}

void PlaneClusterPolicy::InitMap(const int* srcmap, const vecSegment* segments, const cl_float4* normals
                                 , const int emptyID, const int invalidID, int* dstmap)
{
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        if(clIsNull(normals[i]))
            dstmap[i] = invalidID;
        else
            dstmap[i] = emptyID;
    }
}

int PlaneClusterPolicy::GetStartingID(const vecSegment* segments)
{
    if(segments==nullptr)
        return 0;
    int start=0;
    for(auto& seg : *segments)
        start = smax(start, seg.id);
    return start+1;
}

bool PlaneClusterPolicy::IsIgnorable(const Segment& segment)
{
    static const int numptsLowLimit = 30;
    if(segment.numpt <= numptsLowLimit)
        return true;
    else
        return false;
}

bool PlaneClusterPolicy::IsConnected(const Segment& segment, const cl_float4& point, const cl_float4& normal, const cl_float4& nouse)
{
    const float depth = DEPTH(point);
    const float distDiffUppLimit = smin(smax(depth*depth*0.004f, 0.003f), 0.02f);
    if(clNormalDistance(segment.normal, segment.center, point) > distDiffUppLimit)
        return false;

    const float normalAngleThreshDegree = smin(smax(depth*5.f, 3.f), 10.f);
    if(clAngleBetweenVectorsLargerThan(segment.normal, normal, normalAngleThreshDegree, true))
        return false;

    return true;
}

int PlaneClusterPolicy::NextUpdateClusterSize(int currSize)
{
    static const int minUpdateInterval = 5;
    static const float updateRatio = 1.3f;
    return smax(currSize+minUpdateInterval, (int)((float)currSize*updateRatio));
}














