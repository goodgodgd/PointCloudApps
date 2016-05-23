#ifndef SMALLPLANEMERGER_H
#define SMALLPLANEMERGER_H

#include "objectclusterbase.h"

class SmallPlaneMerger : public ObjectClusterBase
{
public:
    SmallPlaneMerger();

private:
    virtual void MergePlanes();
    inline void AbsorbTinyPlanes();
    bool CanAbsorbSmallPlane(const Segment& largerPlane, const Segment& smallerPlane);
    bool IncludeTinyPlane(const Segment& largerPlane, const Segment& smallerPlane);
};

#endif // SMALLPLANEMERGER_H
