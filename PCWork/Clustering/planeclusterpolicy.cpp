#include "planeclusterpolicy.h"

PlaneClusterPolicy::PlaneClusterPolicy()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
{
}

void PlaneClusterPolicy::SetPointCloud(const cl_float4* srcPointCloud)
{
    pointCloud = srcPointCloud;
}

void PlaneClusterPolicy::SetNormalCloud(const cl_float4* srcNormalCloud)
{
    normalCloud = srcNormalCloud;
}

void PlaneClusterPolicy::SetDescriptorCloud(const cl_float4* nouse)
{
}

void PlaneClusterPolicy::InitMap(const int* srcmap, const vecSegment* segments
                                 , const int emptyID, const int invalidID, int* dstmap)
{
    assert(normalCloud!=nullptr);
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        if(clIsNull(normalCloud[i]))
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

bool PlaneClusterPolicy::IsConnected(const Segment& segment, const cl_int2& checkpx)
{
    const int chkidx = PIXIDX(checkpx);
    const float depth = DEPTH(pointCloud[chkidx]);
    const float distDiffUppLimit = smin(smax(depth*depth*0.004f, 0.003f), 0.02f);
    if(clNormalDistance(segment.normal, segment.center, pointCloud[chkidx]) > distDiffUppLimit)
        return false;

    const float normalAngleThreshDegree = smin(smax(depth*5.f, 3.f), 10.f);
    if(clAngleBetweenVectorsLargerThan(segment.normal, normalCloud[chkidx], normalAngleThreshDegree, true))
        return false;

    return true;
}

int PlaneClusterPolicy::NextUpdateClusterSize(int currSize)
{
    static const int minUpdateInterval = 5;
    static const float updateRatio = 1.3f;
    return smax(currSize+minUpdateInterval, (int)((float)currSize*updateRatio));
}

int PlaneClusterPolicy::GetOverlapMargin()
{
    return 1;
}

bool PlaneClusterPolicy::IsFeasibleToMerge(Segment& segL, Segment& segR, int* segmap)
{
    //
    //
    //
    return true;
}












