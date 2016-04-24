#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <cassert>
#include "Share/project_common.h"
#include "Share/shareddata.h"
#include "ClUtils/cloperators.h"
#include "segment.h"
#include "planeclusterpolicy.h"
#include "Share/sharedfunctions.h"

template<typename Policy>
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
    cl_int* Cluster(SharedData* shdDat, const cl_int* srcClusterMap);
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

    Policy clusterPolicy;
    cl_int* segmentMap;
    vecSegment segments;
};

template<typename Policy>
Clusterer<Policy>::Clusterer()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
{
    segmentMap = new int[IMAGE_HEIGHT*IMAGE_WIDTH];
}

template<typename Policy>
cl_int* Clusterer<Policy>::Cluster(SharedData* shdDat, const cl_int* srcClusterMap)
{
    // set class-wide variables
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();

    clusterPolicy.SetData(shdDat, srcClusterMap);
    clusterPolicy.InitMap(MAP_EMPTY, MAP_INVALID, segmentMap);

    FirstClustering(segmentMap, segments);
    qDebug() << "First clustering result:" << segments.size();

    MergeSimiliarSegments(segments);

    return segmentMap;
}

template<typename Policy>
void Clusterer<Policy>::FirstClustering(int* segmentMap, vecSegment& segments)
{
    segments.clear();
    Segment segment;
    cl_int2 pixel;
    int i;
    int segID = 0;

    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            pixel = (cl_int2){x,y};
            i = IMGIDX(y,x);
            if(segmentMap[i]!=MAP_EMPTY || nullityMap[i]>=NullID::NormalNull)
                continue;

            segment.InitSegment(segID, pixel, pointCloud[i], normalCloud[i]);
            FindConnectedComps(segment, pixel);

            if(clusterPolicy.IsIgnorable(segment))
                FillSegment(segment, MAP_EMPTY);
            else
            {
                segments.push_back(segment);
                segID++;
            }
        }
    }
}

template<typename Policy>
int Clusterer<Policy>::FillSegment(const Segment& segment, int neoID)
{
    for(int y=segment.rect.yl; y<=segment.rect.yh; y++)
    {
        for(int x=segment.rect.xl; x<=segment.rect.xh; x++)
        {
            if(segmentMap[IMGIDX(y,x)]==segment.id)
                segmentMap[IMGIDX(y,x)] = neoID;
        }
    }
}

template<typename Policy>
void Clusterer<Policy>::FindConnectedComps(Segment& segment, const cl_int2& checkpx)
{
    const int chkidx = PIXIDX(checkpx);
    if(segmentMap[chkidx]!=MAP_EMPTY)
        return;
    if(clusterPolicy.IsConnected(segment, checkpx)==false)
        return;

    // add pixel to segment
    segmentMap[chkidx] = segment.id;
    // update numpts, center, normal, rect
    UpdateSegment(segment, checkpx);

    // find other connected components
    static const cl_int2 pxmove[] = {{1,0}, {-1,0}, {0,1}, {0,-1}};
    for(int i=0; i<4; i++)
        FindConnectedComps(segment, checkpx+pxmove[i]);
}

template<typename Policy>
void Clusterer<Policy>::UpdateSegment(Segment& segment, const cl_int2& checkpx)
{
    segment.UpdateRect(checkpx);
    segment.numpt++;
    if(segment.numpt != segment.updateAt)
        return;

    segment.updateAt = clusterPolicy.NextUpdateClusterSize(segment.updateAt);
    int pxidx;
    cl_float4 sumpoint = (cl_float4){0.f,0.f,0.f,0.f};
    cl_float4 sumnormal = (cl_float4){0.f,0.f,0.f,0.f};

    // compute mean point and normal
    for(int y=segment.rect.yl; y<=segment.rect.yh; y++)
    {
        for(int x=segment.rect.xl; x<=segment.rect.xh; x++)
        {
            pxidx = IMGIDX(y,x);
            if(segmentMap[pxidx]!=segment.id)
                continue;
            sumpoint = sumpoint + pointCloud[pxidx];
            if(nullityMap[pxidx]<NullID::NormalNull)
                sumnormal = sumnormal + normalCloud[pxidx];
        }
    }

    segment.center = sumpoint/(float)segment.numpt;
    segment.normal = clNormalize(sumnormal);
}

template<typename Policy>
void Clusterer<Policy>::MergeSimiliarSegments(vecSegment& segments)
{
    bool b_merged = false;
    for(auto ri = segments.begin(); ri+1 != segments.end(); ri++)
    {
        auto ci = ri+1;
        do
        {
            b_merged = false;
            for(; ci != segments.end(); ci++)
            {
                if(ri==ci)
                    continue;
                if(ci->id==SEG_INVALID)
                    continue;
                if(clusterPolicy.CanSegmentsBeMerged(*ri, *ci, segmentMap))
                    MergeSegments(*ri, *ci);
                if(ri->id==SEG_INVALID)
                {
                    b_merged = false;
                    break;
                }
                if(ci->id==SEG_INVALID)
                {
                    b_merged = true;
                    ci = segments.begin();
                }
            }
        } while(b_merged);
    }
}

template<typename Policy>
void Clusterer<Policy>::MergeSegments(Segment& segL, Segment& segR)
{
    ImRect mergedRect;
    mergedRect.xl = smin(segL.rect.xl, segR.rect.xl);
    mergedRect.xh = smax(segL.rect.xh, segR.rect.xh);
    mergedRect.yl = smin(segL.rect.yl, segR.rect.yl);
    mergedRect.yh = smax(segL.rect.yh, segR.rect.yh);
    if(segL.numpt < segR.numpt)
    {
        segR.rect = mergedRect;
        FillSegment(segL, segR.id);
        segL.id = SEG_INVALID;
    }
    else
    {
        segL.rect = mergedRect;
        FillSegment(segR, segL.id);
        segR.id = SEG_INVALID;
    }
}

template<typename Policy>
const int* Clusterer<Policy>::GetSegmentMap()
{
    return segmentMap;
}

template<typename Policy>
const vecSegment& Clusterer<Policy>::GetSegments()
{
    return segments;
}

#endif // CLUSTERER_H
