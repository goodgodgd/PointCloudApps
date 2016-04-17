#ifndef PLANECLUSTERPOLICY_H
#define PLANECLUSTERPOLICY_H

#include <cassert>
#include "Share/project_common.h"
#include "ClUtils/cloperators.h"
#include "segment.h"

class PlaneClusterPolicy
{
public:
    PlaneClusterPolicy();
    void SetPointCloud(const cl_float4* srcPointCloud);
    void SetNormalCloud(const cl_float4* srcNormalCloud);
    void SetDescriptorCloud(const cl_float4* nouse);
    void InitMap(const int* srcmap, const vecSegment* segments
                 , const int emptyID, const int invalidID, int* dstmap);
    int GetStartingID(const vecSegment* segments);
    bool IsIgnorable(const Segment& segment);
    bool IsConnected(const Segment& segment, const cl_int2& checkpx);
    int NextUpdateClusterSize(int currSize);
    int GetOverlapMargin();
    bool IsFeasibleToMerge(Segment& segL, Segment& segR, int* segmap);

private:
    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
};

#endif // PLANECLUSTERPOLICY_H
