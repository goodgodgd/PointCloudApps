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
        if(ref->id==MERGED_PLANE)
            continue;
        for(auto cmp=planes.rbegin(); cmp->id!=ref->id; ++cmp)
        {
            if(cmp->id==MERGED_PLANE)
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

    Rangef heightRange = HeightFromPlane<Rangef>(smallerPlane, largerPlane);
    const float heightUpLimit = smax(DEPTH(smallerPlane.center)*DEPTH(smallerPlane.center)*0.01f, 0.005f);
    if(heightRange.high > heightUpLimit || heightRange.low < -heightUpLimit)
        return false;
    return true;
}
