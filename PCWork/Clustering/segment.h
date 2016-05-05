#ifndef SEGMENT_H
#define SEGMENT_H

#include <vector>
#include "Share/project_common.h"
#include "Share/imrect.h"

struct Segment
{
    enum Enum
    {
        MAP_FILLED_FROM = 0,
        MAP_EMPTY = -1,
        MAP_INVALID = -2,
        SEG_INVALID = -1
    };

    int id;
    int numpt;
    int updateAt;
    ImRect rect;
    cl_float4 center;
    cl_float4 normal;

    void InitSegment(const int segID, const cl_int2& pixel, const cl_float4& srcPoint, const cl_float4& srcNormal)
    {
        id = segID;
        numpt = 0;
        updateAt = numpt+1;
        rect.xl = rect.xh = pixel.x;
        rect.yl = rect.yh = pixel.y;
        center = srcPoint;
        normal = srcNormal;
    }

    void ExpandRect(const cl_int2& pixel)
    {
        rect.xl = smin(rect.xl, pixel.x);
        rect.xh = smax(rect.xh, pixel.x);
        rect.yl = smin(rect.yl, pixel.y);
        rect.yh = smax(rect.yh, pixel.y);
    }

    void ExpandRect(const ImRect& srcRect)
    {
        rect.xl = smin(rect.xl, srcRect.xl);
        rect.xh = smax(rect.xh, srcRect.xh);
        rect.yl = smin(rect.yl, srcRect.yl);
        rect.yh = smax(rect.yh, srcRect.yh);
    }
};

typedef std::vector<Segment>    vecSegment;

#endif // SEGMENT_H
