#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#undef __STRICT_ANSI__  // without this, cl_float4.x,y,z are not available
#include <CL/cl.h>
#include <QDebug>
#include <QVector3D>

#define IMAGE_WIDTH     320
#define IMAGE_HEIGHT    240
#define DEPTH_RANGE     3500
#define NEIGHBORS_PER_POINT 30

#define PI_F            3.14159265359f

#define smax(a,b)       ((a>b)?a:b)
#define smin(a,b)       ((a<b)?a:b)

extern int g_frameIdx;

typedef cl_float4 DescType;

#endif // PROJECT_COMMON_H
