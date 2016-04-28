#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <cassert>
#include "Share/project_common.h"
#include "Share/shareddata.h"
#include "Share/arraydata.h"
#include "ClUtils/cloperators.h"
#include "segment.h"
#include "planeclusterpolicy.h"

template<typename Policy>
class Clusterer
{
    typedef std::vector<cl_int>    vecInt;

public:
    Clusterer();
    void Cluster(SharedData* shdDat);
    const cl_int* GetSegmentMap();
    const vecSegment* GetSegments();

private:
    void DoClustering(cl_int* segmentMap, vecSegment& segments);
    int FillSegment(const Segment& segment, int neoID);
    void FindConnectedComps(Segment& segment, const cl_int2& checkpx);
    void UpdateSegment(Segment& segment, const cl_int2& checkpx);
    void AbsorbSmallBlobs(cl_int* segmentMap, vecSegment& segments);
    void ErodeEmptyArea(cl_int* srcmap, vecInt& srcIndices, vecSegment& segments, cl_int* dstmap, vecInt& dstIndices);

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;

    Policy clusterPolicy;
    ArrayData<cl_int> segmentArray;
    cl_int* segmentMap;
    vecSegment segments;
};

template<typename Policy>
Clusterer<Policy>::Clusterer()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
{
    segmentArray.Allocate(IMAGE_HEIGHT*IMAGE_WIDTH);
    segmentMap = segmentArray.GetArrayPtr();
}

template<typename Policy>
void Clusterer<Policy>::Cluster(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();

    clusterPolicy.SetData(shdDat);
    clusterPolicy.InitMap(Segment::MAP_EMPTY, Segment::MAP_INVALID, segmentMap);

    DoClustering(segmentMap, segments);
    qDebug() << "   # clusters =" << segments.size();

    AbsorbSmallBlobs(segmentMap, segments);
}

template<typename Policy>
void Clusterer<Policy>::DoClustering(cl_int* segmentMap, vecSegment& segments)
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
            if(segmentMap[i]!=Segment::MAP_EMPTY || nullityMap[i]>=NullID::NormalNull)
                continue;

            segment.InitSegment(segID, pixel, pointCloud[i], normalCloud[i]);
            FindConnectedComps(segment, pixel);

            if(clusterPolicy.IsIgnorable(segment))
                FillSegment(segment, Segment::MAP_EMPTY);
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
    if(OUTSIDEIMG(checkpx.y, checkpx.x))
        return;
    const int chkidx = PIXIDX(checkpx);
    if(segmentMap[chkidx]!=Segment::MAP_EMPTY)
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
void Clusterer<Policy>::AbsorbSmallBlobs(cl_int* segmentMap, vecSegment& segments)
{
    static int tempMap[IMAGE_WIDTH*IMAGE_HEIGHT];
    vecInt srcEmptyIndices;
    vecInt erodeEmptyIndices;

    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
        if(segmentMap[i]==Segment::MAP_EMPTY)
            srcEmptyIndices.push_back(i);

    int cnt=0;
    do {
        memcpy(tempMap, segmentMap, sizeof(cl_int)*IMAGE_WIDTH*IMAGE_HEIGHT);
        ErodeEmptyArea(tempMap, srcEmptyIndices, segments, segmentMap, erodeEmptyIndices);
        srcEmptyIndices.swap(erodeEmptyIndices);
//        qDebug() << "absorb" << erodeEmptyIndices.size();
    } while(srcEmptyIndices.empty()==false && srcEmptyIndices.size()!=erodeEmptyIndices.size());
}

template<typename Policy>
void Clusterer<Policy>::ErodeEmptyArea(cl_int* srcmap, vecInt& srcIndices, vecSegment& segments, cl_int* dstmap, vecInt& dstIndices)
{
    static const cl_int2 pxmove[] = {{0,-1}, {0,1}, {1,0}, {-1,0}};
    cl_int2 neigh;
    int x, y, mi, neidx, segID;
    dstIndices.clear();

    for(int pxidx : srcIndices)
    {
        y = pxidx/IMAGE_WIDTH;
        x = pxidx%IMAGE_WIDTH;
        if(srcmap[pxidx]!=Segment::MAP_EMPTY)
            continue;

        for(mi=0; mi<4; mi++)
        {
            neigh = (cl_int2){x + pxmove[mi].x, y + pxmove[mi].y};
            if(OUTSIDEIMG(neigh.y, neigh.x))
                continue;
            neidx = IMGIDX(neigh.y, neigh.x);
            if(srcmap[neidx]>=Segment::MAP_FILLED_FROM)
            {
                segID = srcmap[neidx];
                dstmap[pxidx] = segID;
                assert(segID==segments[segID].id);
                UpdateSegment(segments[segID], neigh);
                break;
            }
        }
        if(mi==4)
            dstIndices.push_back(pxidx);
    }
}


template<typename Policy>
const cl_int* Clusterer<Policy>::GetSegmentMap()
{
    return segmentMap;
}

template<typename Policy>
const vecSegment* Clusterer<Policy>::GetSegments()
{
    return &segments;
}

#endif // CLUSTERER_H
