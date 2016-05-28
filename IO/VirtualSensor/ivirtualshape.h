#ifndef IVIRTUALSHAPE_H
#define IVIRTUALSHAPE_H

#include <cassert>
#include "Share/project_common.h"
#include "Share/shared_types.h"
#include "Share/range.h"
#include "ClUtils/cloperators.h"

struct IVirtualShape
{
    enum Enum
    {
        RECT,
        SPHERE
    };

    // virtual descturctor must be declared in interface class
    virtual ~IVirtualShape() {}
    // =0; means derived classes must implement the function
    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersect) const = 0;
    int type;

protected:
    cl_float4 center;
};

#endif // IVIRTUALSHAPE_H
