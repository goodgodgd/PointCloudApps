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

    Segment& operator=(const Segment& srcseg)
    {
        this->id = srcseg.id;
        this->numpt = srcseg.numpt;
        this->updateAt = srcseg.updateAt;
        this->rect = srcseg.rect;
        this->center = srcseg.center;
        this->normal = srcseg.normal;
    }
};

typedef std::vector<Segment>        vecSegment;
typedef std::pair<Segment,Segment>  PairOfSegment;

#endif // SEGMENT_H
