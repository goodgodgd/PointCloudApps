#include "clusterer.h"

Clusterer::Clusterer()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , descriptorCloud(nullptr)
    , segmap(nullptr)
{
    segmap = new int[IMAGE_HEIGHT*IMAGE_WIDTH];
}

void Clusterer::ClusterPointCloud(const cl_float4* srcPointCloud, const cl_float4* srcNormalCloud, const cl_float4* srcDescriptorCloud
                       , const int* srcMap, const vecSegment* srcSegments)
{
    // set class-wide variables
    pointCloud = srcPointCloud;
    normalCloud = srcNormalCloud;
    descriptorCloud = srcDescriptorCloud;
    clusterPolicy.SetPointCloud(srcPointCloud);
    clusterPolicy.SetNormalCloud(srcNormalCloud);
    clusterPolicy.SetDescriptorCloud(srcDescriptorCloud);

    clusterPolicy.InitMap(srcMap, srcSegments, MAP_EMPTY, MAP_INVALID, segmap);
    int startID = clusterPolicy.GetStartingID(srcSegments);
    FirstClustering(startID, segmap, segments);
    qDebug() << "First clustering:" << segments.size();

    return;
    // merge adjacent clusters by comparing interfacing pixels
    MergeAdjacentSimiliarClusters(segments);

    // expand planes by only normal distance from the plane NOT by normal angle from the LARGEST planes
}

void Clusterer::FirstClustering(const int startID, int* segmap, vecSegment& segments)
{
    segments.clear();
    Segment segment;
    cl_int2 pixel;
    int i;
    int segID = startID;

    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            pixel = (cl_int2){x,y};
            i = IMGIDX(y,x);
            if(segmap[i]!=MAP_EMPTY)
                continue;

            segment.InitSegment(segID, pixel, pointCloud[i], normalCloud[i]);
            FindConnectedComps(segment, pixel);

            if(clusterPolicy.IsIgnorable(segment))
                FillSegment(segment, MAP_EMPTY);// MAP_IGNORE);
            else
            {
                segments.push_back(segment);
                segID++;
            }
        }
    }
}

int Clusterer::FillSegment(const Segment& segment, int neoID)
{
    for(int y=segment.rect.yl; y<=segment.rect.yh; y++)
    {
        for(int x=segment.rect.xl; x<=segment.rect.xh; x++)
        {
            if(segmap[IMGIDX(y,x)]==segment.id)
                segmap[IMGIDX(y,x)] = neoID;
        }
    }
}

void Clusterer::FindConnectedComps(Segment& segment, const cl_int2& checkpx)
{
    const int chkidx = PIXIDX(checkpx);
    if(segmap[chkidx]==MAP_INVALID)
        return;
    if(segmap[chkidx]!=MAP_EMPTY)
        return;
    if(clusterPolicy.IsConnected(segment, checkpx)==false)
        return;

    // add pixel to segment
    segmap[chkidx] = segment.id;
    // update numpts, center, normal, rect
    UpdateSegment(segment, checkpx);

    // find other connected components
    static const cl_int2 pxmove[] = {{1,0}, {-1,0}, {0,1}, {0,-1}};
    for(int i=0; i<4; i++)
        FindConnectedComps(segment, checkpx+pxmove[i]);
}

void Clusterer::UpdateSegment(Segment& segment, const cl_int2& checkpx)
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
            if(segmap[pxidx]!=segment.id)
                continue;
            sumpoint = sumpoint + pointCloud[pxidx];
            sumnormal = sumnormal + normalCloud[pxidx];
        }
    }

    segment.center = sumpoint/(float)segment.numpt;
    segment.normal = clNormalize(sumnormal);
}

void Clusterer::MergeAdjacentSimiliarClusters(vecSegment& segments)
{
    for(auto ri = segments.begin(); ri+1 != segments.end(); ri++)
    {
        for(auto ci = ri+1; ci != segments.end(); ci++)
        {
            if(CheckRectsOverlap(ri->rect, ci->rect, clusterPolicy.GetOverlapMargin()))
                if(clusterPolicy.IsFeasibleToMerge(*ri, *ci, segmap))
                    MergeClusters(*ri, *ci);
        }
    }
}

bool Clusterer::CheckRectsOverlap(const ImRect& rectL, const ImRect& rectR, const int margin)
{
    return (rectL.xh+margin < rectR.xl-margin || rectR.xh+margin < rectL.xl-margin
             || rectL.yh+margin < rectR.yl-margin || rectR.yh+margin < rectL.yl-margin);
}

void Clusterer::MergeClusters(Segment& segL, Segment& segR)
{
    //
    //
    //
    Segment ovlseg;
    ovlseg.rect.xl = smax(segL.rect.xl, segR.rect.xl);
    ovlseg.rect.xh = smin(segL.rect.xh, segR.rect.xh);
    ovlseg.rect.yl = smax(segL.rect.yl, segR.rect.yl);
    ovlseg.rect.yh = smin(segL.rect.xh, segR.rect.yh);
}

const int* Clusterer::GetSegmentMap()
{
    return segmap;
}

const vecSegment& Clusterer::GetSegments()
{
    return segments;
}











