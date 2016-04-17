#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <cassert>
#include "Share/project_common.h"
#include "ClUtils/cloperators.h"
#include "segment.h"
#include "planeclusterpolicy.h"

class Clusterer
{
    enum MapID
    {
        MAP_EMPTY = -1,
        MAP_INVALID = -2,
        MAP_IGNORE = -3
    };

public:
    Clusterer();
    void ClusterPointCloud(const cl_float4* srcPointCloud, const cl_float4* srcNormalCloud, const cl_float4* srcDescriptorCloud
                           , const int* srcMap, const vecSegment* srcSegments);
    const int* GetSegmentMap();
    const vecSegment& GetSegments();

private:
    void FirstClustering(const int startID, int* segmap, vecSegment& segments);
    int FillSegment(const Segment& segment, int neoID);
    void FindConnectedComps(Segment& segment, const cl_int2& checkpx);
    void MergeSegments(int srcSegIdx, int dstSegIdx);
    void UpdateSegment(Segment& segment, const cl_int2& checkpx);
    void MergeAdjacentSimiliarClusters(vecSegment& segments);
    inline bool CheckRectsOverlap(const ImRect& rectL, const ImRect& rectR, const int margin);
    void MergeClusters(Segment& segL, Segment& segR);

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_float4* descriptorCloud;
    const int* preMap;
    const vecSegment* preSegments;

    PlaneClusterPolicy clusterPolicy;
    int* segmap;
    vecSegment segments;
};

#endif // CLUSTERER_H
