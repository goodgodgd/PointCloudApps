#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <cassert>
#include "Share/project_common.h"
#include "Share/shareddata.h"
#include "ClUtils/cloperators.h"
#include "segment.h"
#include "planeclusterpolicy.h"
#include "Share/sharedfunctions.h"

class Clusterer
{
    enum Enum
    {
        MAP_FILLED_FROM = 0,
        MAP_EMPTY = -1,
        MAP_INVALID = -2,
        SEG_INVALID = -1
    };

    typedef std::vector<int>    vecInt;

public:
    Clusterer();
    void Cluster(SharedData* shdDat, const cl_int* srcClusterMap);
    const int* GetSegmentMap();
    const vecSegment& GetSegments();

private:
    void FirstClustering(int* segmentMap, vecSegment& segments);
    int FillSegment(const Segment& segment, int neoID);
    void FindConnectedComps(Segment& segment, const cl_int2& checkpx);
    void MergeSegments(int srcSegIdx, int dstSegIdx);
    void UpdateSegment(Segment& segment, const cl_int2& checkpx);
    void MergeSimiliarSegments(vecSegment& segments);
    inline ImRect FindOverlapRect(const ImRect& rectL, const ImRect& rectR, const int margin);
    void MergeSegments(Segment& segL, Segment& segR);

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;

    PlaneClusterPolicy clusterPolicy;
    int* segmentMap;
    vecSegment segments;
};

#endif // CLUSTERER_H
