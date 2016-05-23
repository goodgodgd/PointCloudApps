#ifndef TESTINOBJCLUSTER_H
#define TESTINOBJCLUSTER_H

#include <utility>
#include <cassert>
#include <QImage>
#include "Share/project_common.h"
#include "Share/shared_data.h"
#include "Share/arraydata.h"
#include "Share/shared_enums.h"
#include "Share/camera_param.h"
#include "ClUtils/cloperators.h"
#include "PCWork/Clustering/segment.h"

inline bool TestPlaneIntersect(const Segment& leftPlane, const Segment& rightPlane, const PairPointNormal& intersectLine)
{
    const cl_float4& linePoint = intersectLine.first;
    const cl_float4& lineDir = intersectLine.second;
    if(fabsf(clDot(leftPlane.normal, leftPlane.center) - clDot(leftPlane.normal, linePoint)) > 0.001f)
        return false;
    if(fabsf(clDot(rightPlane.normal, rightPlane.center) - clDot(rightPlane.normal, linePoint)) > 0.001f)
        return false;
    if(fabsf(clDot(leftPlane.normal, lineDir)) > 0.001f)
        return false;
    if(fabsf(clDot(rightPlane.normal, lineDir)) > 0.001f)
        return false;
    return true;
}

#endif // TESTINOBJCLUSTER_H
