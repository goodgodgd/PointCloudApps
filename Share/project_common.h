#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#undef __STRICT_ANSI__  // without this, cl_float4.x,y,z are not available
#include <CL/cl.h>
#include <QDebug>
#include <QVector3D>

#define IMAGE_WIDTH     320
#define IMAGE_HEIGHT    240
#define DEPTH_RANGE_MM  3500
#define DEAD_RANGE_M    0.1f

#define PI_F            3.14159265359f

#define smax(a,b)       ((a>b)?a:b)
#define smin(a,b)       ((a<b)?a:b)
#define IMGIDX(y,x)     (y*IMAGE_WIDTH+x)

extern int g_frameIdx;

#endif // PROJECT_COMMON_H
