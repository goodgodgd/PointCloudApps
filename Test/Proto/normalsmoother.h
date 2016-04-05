#ifndef POINTNORMALSMOOTHER_H
#define POINTNORMALSMOOTHER_H

#include "Share/project_common.h"
#include "ClUtils/cloperators.h"
#include "indexsort.h"

class NormalSmoother
{
#define ADJ_SIZE    7
#define MG          2

public:
    NormalSmoother();
    void SmootheNormalCloud(cl_float4* pointCloud, cl_float4* normalCloud);
private:
    int GetAdjacentPixels(cl_int2 centerPixel, cl_int2* adjacentPixels);
    inline cl_float4 AverageNormals(cl_int2* pixels, const int num);
    int ExtractInliers(cl_int2 centerPixel, cl_int2* adjacentPixels, const int numAdj);
    inline bool AngleBetweenVectorsLessThan(const cl_float4& v1, const cl_float4& v2, const float degree, bool b_normalized);
    inline void Swap(cl_int2& foo, cl_int2& bar);

    cl_float4* pointCloud;
    cl_float4* normalCloud;
};

#endif // POINTNORMALSMOOTHER_H
