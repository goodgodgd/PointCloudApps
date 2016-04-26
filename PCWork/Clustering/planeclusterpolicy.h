#ifndef PLANECLUSTERPOLICY_H
#define PLANECLUSTERPOLICY_H

#include "Share/project_common.h"
#include "ClUtils/cloperators.h"
#include "segment.h"

class PlaneClusterPolicy
{
public:
    PlaneClusterPolicy();
    void InitMap(const int* srcmap, const vecSegment* segments, const cl_float4* normals
                 , const int emptyID, const int invalidID, int* dstmap);
    int GetStartingID(const vecSegment* segments);
    bool IsIgnorable(const Segment& segment);
    bool IsConnected(const Segment& segment, const cl_float4& point, const cl_float4& normal, const cl_float4& nouse);
    int NextUpdateClusterSize(int currSize);
};

#endif // PLANECLUSTERPOLICY_H
