#ifndef PLANECLUSTERPOLICY_H
#define PLANECLUSTERPOLICY_H

#include <cassert>
#include "Share/project_common.h"
#include "Share/shareddata.h"
#include "Share/sharedenums.h"
#include "ClUtils/cloperators.h"
#include "segment.h"

class PlaneClusterPolicy
{
#define PLANE_EXTRACT_RANGE_M   1.5f
public:
    PlaneClusterPolicy();
    void SetData(SharedData* shdDat, const cl_int* srcMap=nullptr, const vecSegment* srcSegments=nullptr);
    void InitMap(const int emptyID, const int invalidID, int* dstmap);
    bool IsIgnorable(const Segment& segment);
    bool IsConnected(const Segment& segment, const cl_int2& checkpx);
    int NextUpdateClusterSize(int currSize);
    int GetOverlapMargin();
    void PostProcess(int* segmentMap, vecSegment& segments, const int absorbID);

private:
    ImRect FindOverlapRect(const ImRect& rectL, const ImRect& rectR, const int margin);
    bool CanSegmentsBeMerged(Segment& segL, Segment& segR, int* segmap);
    bool CanPixelsBeMerged(int leftidx, int rightidx, const cl_float4& segNormal);
    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;
};

#endif // PLANECLUSTERPOLICY_H
