#include "smallplanemerger.h"

SmallPlaneMerger::SmallPlaneMerger()
{
    qDebug() << "SmallPlaneMerger";
}

void SmallPlaneMerger::MergePlanes()
{
    AbsorbTinyPlanes();
}

void SmallPlaneMerger::AbsorbTinyPlanes()
{
    const int smallLimit = 100;
    for(auto ref=planes.begin(); ref+1!=planes.end(); ++ref)
    {
        if(ref->id==MERGED_PLANE)
            continue;
        for(auto cmp=planes.rbegin(); cmp->id!=ref->id; ++cmp)
        {
            if(cmp->id==MERGED_PLANE)
                continue;
            if(cmp->numpt > smallLimit)
                break;
            if(CanAbsorbSmallPlane(*ref, *cmp))
                AbsorbPlane(*ref, *cmp);
        }
    }
}

bool SmallPlaneMerger::CanAbsorbSmallPlane(const Segment& largerPlane, const Segment& smallerPlane)
{
    assert(largerPlane.numpt >= smallerPlane.numpt);
    if(DoRectsOverlap(largerPlane.rect, smallerPlane.rect)==false)
        return false;
    ImRect ovlRect = OverlappingRect(largerPlane.rect, smallerPlane.rect);

    vecPairOfPixels connPixels;
    if(ArePlanesConnected(ovlRect, largerPlane, smallerPlane, connPixels)==false)
        return false;

    return IncludeTinyPlane(largerPlane, smallerPlane);
}

bool SmallPlaneMerger::IncludeTinyPlane(const Segment& largerPlane, const Segment& smallerPlane)
{
    assert(clDot(largerPlane.center, largerPlane.normal) < 0.f);
    assert(fabsf(clLength(largerPlane.normal)-1.f) < 0.0001f);

    const float planeDist = -clDot(largerPlane.center, largerPlane.normal);
    const float heightUpLimit = smax(DEPTH(smallerPlane.center)*DEPTH(smallerPlane.center)*0.01f, 0.005f);
    float minNormalDist=10000.f;
    const ImRect& range = smallerPlane.rect;
    for(int y=range.yl; y<=range.yh; y++)
    {
        for(int x=range.xl; x<=range.xh; x++)
        {
            if(objectMap[IMGIDX(y,x)]==smallerPlane.id)
                minNormalDist = smin(minNormalDist, -clDot(pointCloud[IMGIDX(y,x)], largerPlane.normal));
        }
    }
//    qDebug() << "includeTiny" << largerPlane.id << largerPlane.numpt << smallerPlane.id << smallerPlane.numpt
//             << "height" << planeDist << planeDist - minNormalDist << heightUpLimit << (planeDist - minNormalDist < heightUpLimit);

    // include tiny neighbor plane except for hightly convex to large plane
    return (planeDist - minNormalDist < heightUpLimit);
}
