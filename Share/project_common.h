#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#undef __STRICT_ANSI__  // without this, cl_float4.x,y,z are not available
#include <QDebug>
#include <QVector3D>
#include <CL/cl.h>
#include <stdio.h>

#define IMAGE_WIDTH     320
#define IMAGE_HEIGHT    240

#define PI_F            3.14159265359f

#define smax(a,b)       ((a>b)?a:b)
#define smin(a,b)       ((a<b)?a:b)
#define IMGIDX(y,x)     ((y)*IMAGE_WIDTH+(x))
#define PIXIDX(p)       ((p.y)*IMAGE_WIDTH+(p.x))
#define OUTSIDEIMG(y,x) ((x) < 0 || (x) >= IMAGE_WIDTH || (y) < 0 || (y) >= IMAGE_HEIGHT)
#define INSIDEIMG(y,x)  ((x) >= 0 && (x) < IMAGE_WIDTH && (y) >= 0 && (y) < IMAGE_HEIGHT)
#define DEPTH(p)        (p).x
#define DEG2RAD(r)      (r)*PI_F/180.f
#define RAD2DEG(r)      (r)*180.f/PI_F

extern int g_frameIdx;

#endif // PROJECT_COMMON_H
