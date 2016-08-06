#ifndef POINTSMOOTHER_H
#define POINTSMOOTHER_H

#include "Share/project_common.h"
#include "ClUtils/cloperators.h"

class PointSmoother
{
#define MG      2

public:
    PointSmoother();
    static void SmoothePointCloud(cl_float4* pointCloud, cl_float4* normalCloud);
    static inline cl_float4 SmoothePoint(cl_float4* pointCloud, const cl_int2& pixel, cl_float4& normal);
    static cl_int2 SearchLinearDirection(cl_float4* pointCloud, const cl_int2& pixel, cl_float4& normal);
    static cl_float4 SmoothePointByMeanDepth(cl_float4* pointCloud, const cl_int2& pixel, const cl_int2& linearDir);
};

#endif // POINTSMOOTHER_H
