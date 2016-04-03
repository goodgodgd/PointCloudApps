#ifndef CLOPERATORS_H
#define CLOPERATORS_H

#include <QImage>
#include "Share/project_common.h"

inline cl_float4 clNormalize(const cl_float4& src);
inline float clDot(const cl_float4& v1, const cl_float4& v2);
inline bool clIsNull(const cl_float4& src, const float precision=0.0001f);


//////////////////// BASIC ARITHMATIC OPERATORS ////////////////////

inline cl_float4 operator +(const cl_float4& src, const float val)
{
    cl_float4 dst;
    dst.x = src.x + val;
    dst.y = src.y + val;
    dst.z = src.z + val;
    dst.w = src.w + val;
    return dst;
}

inline cl_float4 operator +(const cl_float4& v1, const cl_float4 v2)
{
    cl_float4 dst;
    dst.x = v1.x + v2.x;
    dst.y = v1.y + v2.y;
    dst.z = v1.z + v2.z;
    dst.w = v1.w + v2.w;
    return dst;
}

inline cl_float4 operator -(const cl_float4& src, const float val)
{
    cl_float4 dst;
    dst.x = src.x - val;
    dst.y = src.y - val;
    dst.z = src.z - val;
    dst.w = src.w - val;
    return dst;
}

inline cl_float4 operator -(const cl_float4& v1, const cl_float4 v2)
{
    cl_float4 dst;
    dst.x = v1.x - v2.x;
    dst.y = v1.y - v2.y;
    dst.z = v1.z - v2.z;
    dst.w = v1.w - v2.w;
    return dst;
}

inline cl_float4 operator *(const cl_float4& src, const float val)
{
    cl_float4 dst;
    dst.x = src.x * val;
    dst.y = src.y * val;
    dst.z = src.z * val;
    dst.w = src.w * val;
    return dst;
}

inline cl_float4 operator /(const cl_float4& src, const float val)
{
    cl_float4 dst;
    dst.x = src.x / val;
    dst.y = src.y / val;
    dst.z = src.z / val;
    dst.w = src.w / val;
    return dst;
}

inline cl_int2 operator +(const cl_int2& v1, const cl_int2 v2)
{
    cl_int2 dst;
    dst.x = v1.x + v2.x;
    dst.y = v1.y + v2.y;
    return dst;
}

inline cl_int2 operator -(const cl_int2& v1, const cl_int2 v2)
{
    cl_int2 dst;
    dst.x = v1.x - v2.x;
    dst.y = v1.y - v2.y;
    return dst;
}

//////////////////// STREAM OPERATORS ////////////////////

inline cl_float4& operator <<(cl_float4& dstvec, QVector3D& srcvec)
{
    dstvec = (cl_float4){srcvec.x(), srcvec.y(), srcvec.z(), 0.f};
    return dstvec;
}

inline QVector3D& operator <<(QVector3D& dstvec, cl_float4& srcvec)
{
    dstvec = QVector3D(srcvec.x, srcvec.y, srcvec.z);
    return dstvec;
}

inline QDebug operator <<(QDebug debug, const cl_float4 &c)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "clf4(" << c.x << ", " << c.y << ", " << c.z << ", " << c.w << ')';
    return debug.space();
}

inline cl_float4& operator <<(cl_float4& dstColor, QRgb& srcColor)
{
    dstColor = (cl_float4){(float)qRed(srcColor)/256.f, (float)qGreen(srcColor)/256.f, (float)qBlue(srcColor)/256.f, 0.f};
    return dstColor;
}

//////////////////// BASIC FUNCTIONS ////////////////////

inline cl_float4 clNormalize(const cl_float4& src)
{
    cl_float4 dst;
    float sqlen = src.x*src.x + src.y*src.y + src.z*src.z + src.w*src.w;
    if(fabsf(sqlen) < 0.0001f)
        return src;
    dst = src / sqrt(sqlen);
    return dst;
}

inline float clDot(const cl_float4& v1, const cl_float4& v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z + v1.w*v2.w;
}

inline bool clIsNull(const cl_float4& src, const float precision)
{
    if(fabsf(src.x)<precision && fabsf(src.y)<precision && fabsf(src.z)<precision/* && fabsf(src.w)<precision*/)
        return true;
    else
        return false;
}

#endif // CLOPERATORS_H
