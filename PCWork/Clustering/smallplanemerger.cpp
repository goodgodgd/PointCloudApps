#include "smallplanemerger.h"

SmallPlaneMerger::SmallPlaneMerger()
{
}

void SmallPlaneMerger::MergePlanes()
{
    AbsorbTinyPlanes();
}

void SmallPlaneMerger::AbsorbTinyPlanes()
{
    for(auto ref=planes.begin(); ref+1!=planes.end(); ++ref)
    {
        if(ref->id==Segment::SEG_INVALID)
            continue;
        for(auto cmp=planes.rbegin(); cmp->id!=ref->id; ++cmp)
        {
            if(cmp->id==Segment::SEG_INVALID)
                continue;
            if(cmp->numpt > SMALL_LIMIT)
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

    float height = HeightFromPlane(smallerPlane, largerPlane, true);
    const float heightUpLimit = smax(DEPTH(smallerPlane.center)*DEPTH(smallerPlane.center)*0.01f, 0.007f);
    return (height < heightUpLimit);
}
