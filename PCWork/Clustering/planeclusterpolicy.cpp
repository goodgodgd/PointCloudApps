#include "planeclusterpolicy.h"

PlaneClusterPolicy::PlaneClusterPolicy()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
{
}

void PlaneClusterPolicy::SetData(SharedData* shdDat, const cl_int* srcMap, const vecSegment* srcSegments)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();
}

void PlaneClusterPolicy::InitMap(const int emptyID, const int invalidID, int* dstmap)
{
    assert(normalCloud!=nullptr);
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        if(nullityMap[i]==NullID::PointNull)
            dstmap[i] = invalidID;
        else if(DEPTH(pointCloud[i]) > PLANE_EXTRACT_RANGE_M)
            dstmap[i] = invalidID;
        else
            dstmap[i] = emptyID;
    }
}

bool PlaneClusterPolicy::IsIgnorable(const Segment& segment)
{
    static const int numptsLowLimit = 10;
    if(segment.numpt <= numptsLowLimit)
        return true;
    else
        return false;
}

bool PlaneClusterPolicy::IsConnected(const Segment& segment, const cl_int2& checkpx)
{
    const int chkidx = PIXIDX(checkpx);
    const float depth = DEPTH(pointCloud[chkidx]);
    const float distDiffUppLimit = smax(depth*depth*0.005f, 0.003f);
    const float normalAngleThreshDegree = smax(depth-0.5f, 0.f)*20.f + 20.f;    // 20deg at 0.5m ~ 40deg at 1.5m

    if(nullityMap[chkidx]<NullID::NormalNull)
        if(clAngleBetweenVectorsLargerThan(segment.normal, normalCloud[chkidx], normalAngleThreshDegree, true))
            return false;
    if(clNormalDistance(segment.normal, segment.center, pointCloud[chkidx]) > distDiffUppLimit)
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
    return 3;
}

bool PlaneClusterPolicy::CanSegmentsBeMerged(Segment& segL, Segment& segR, int* segmap)
{
    ImRect ovlrect = FindOverlapRect(segL.rect, segR.rect, GetOverlapMargin());
    if(ovlrect.xl==ovlrect.xh || ovlrect.yl==ovlrect.yh)
        return false;
    if(clAngleBetweenVectorsLargerThan(segL.normal, segR.normal, 10, true))
        return false;

    int hereidx, rigidx, lowidx;
    int connectCount=0, mergeCount=0;
    for(int y=ovlrect.yl; y<ovlrect.yh; y++)
    {
        for(int x=ovlrect.xl; x<ovlrect.xh; x++)
        {
            hereidx = IMGIDX(y,x);
            rigidx = IMGIDX(y,x+1);
            lowidx = IMGIDX(y+1,x);

            if((segmap[hereidx]==segL.id && segmap[rigidx]==segR.id)
                    || (segmap[hereidx]==segR.id && segmap[rigidx]==segL.id))
            {
                connectCount++;
                if(CanPixelsBeMerged(hereidx, rigidx, segL.normal))
                    mergeCount++;
            }

            if((segmap[hereidx]==segL.id && segmap[lowidx]==segR.id)
                    || (segmap[hereidx]==segR.id && segmap[lowidx]==segL.id))
            {
                connectCount++;
                if(CanPixelsBeMerged(hereidx, lowidx, segL.normal))
                    mergeCount++;
            }
        }
    }

    return (mergeCount > connectCount*0.7f);
}

bool PlaneClusterPolicy::CanPixelsBeMerged(int leftidx, int rightidx, const cl_float4& segNormal)
{
    const float leftDepth = DEPTH(pointCloud[leftidx]);
    const float distDiffUppLimit = smax(leftDepth*leftDepth*0.005f, 0.003f);
    if(fabs(clDot(pointCloud[leftidx] - pointCloud[rightidx], segNormal)) > distDiffUppLimit)
        return false;

    return true;
}

ImRect PlaneClusterPolicy::FindOverlapRect(const ImRect& rectL, const ImRect& rectR, const int margin)
{
    ImRect rect;
    rect.xl = smax(smax(rectL.xl-margin, rectR.xl-margin), 0);
    rect.xh = smin(smin(rectL.xh+margin, rectR.xh+margin), IMAGE_WIDTH-1);
    rect.yl = smax(smax(rectL.yl-margin, rectR.yl-margin), 0);
    rect.yh = smin(smin(rectL.yh+margin, rectR.yh+margin), IMAGE_HEIGHT-1);

    rect.xl = smin(rect.xl, rect.xh);
    rect.yl = smin(rect.yl, rect.yh);
    return rect;
}


