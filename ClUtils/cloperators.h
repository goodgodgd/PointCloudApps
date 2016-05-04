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

inline cl_float4 operator +(const cl_float4& v1, const cl_float4& v2)
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

inline cl_float4 operator -(const cl_float4& v1, const cl_float4& v2)
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

inline cl_int2 operator +(const cl_int2& v1, const cl_int2& v2)
{
    cl_int2 dst;
    dst.x = v1.x + v2.x;
    dst.y = v1.y + v2.y;
    return dst;
}

inline cl_int2 operator -(const cl_int2& v1, const cl_int2& v2)
{
    cl_int2 dst;
    dst.x = v1.x - v2.x;
    dst.y = v1.y - v2.y;
    return dst;
}

//////////////////// STREAM OPERATORS ////////////////////

inline cl_float4& operator <<(cl_float4& dstvec, const QVector3D& srcvec)
{
    dstvec = (cl_float4){srcvec.x(), srcvec.y(), srcvec.z(), 0.f};
    return dstvec;
}

inline QVector3D& operator <<(QVector3D& dstvec, const cl_float4& srcvec)
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

inline QDebug operator <<(QDebug debug, const cl_float2 &c)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "clf2(" << c.x << ", " << c.y << ")";
    return debug.space();
}

inline QDebug operator <<(QDebug debug, const cl_int2 &c)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "cli2(" << c.x << ", " << c.y << ")";
    return debug.space();
}

inline cl_float4& operator <<(cl_float4& dstColor, const QRgb& srcColor)
{
    dstColor = (cl_float4){(float)qRed(srcColor)/256.f, (float)qGreen(srcColor)/256.f, (float)qBlue(srcColor)/256.f, 0.f};
    return dstColor;
}

//////////////////// BASIC FUNCTIONS ////////////////////

inline cl_float4 clNormalize(const cl_float4& src)
{
    cl_float4 dst;
    float sqlen = src.x*src.x + src.y*src.y + src.z*src.z;// + src.w*src.w;
    if(fabsf(sqlen) < 0.0001f)
        return src;
    dst = src / sqrt(sqlen);
    return dst;
}

inline float clDot(const cl_float4& v1, const cl_float4& v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;// + v1.w*v2.w;
}

inline cl_float4 clCross(const cl_float4& v1, const cl_float4& v2)
{
    cl_float4 cross;
    cross.x = v1.y*v2.z - v1.z*v2.y;
    cross.y = v1.z*v2.x - v1.x*v2.z;
    cross.z = v1.x*v2.y - v1.y*v2.x;
    cross.w = 0;
    return cross;
}

inline bool clIsNull(const cl_float4& src, const float precision)
{
    if(fabsf(src.x)<precision && fabsf(src.y)<precision && fabsf(src.z)<precision/* && fabsf(src.w)<precision*/)
        return true;
    else
        return false;
}

inline float clLength(const cl_float4& vec)
{
    float sqlen = vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;// + vec.w*vec.w;
    return sqrt(sqlen);
}

inline float clSqLength(const cl_float4& vec)
{
    return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;// + vec.w*vec.w;
}

inline bool clIsNan(const cl_float4& vec)
{
    if(isnanf(vec.x) || isnanf(vec.y) || isnanf(vec.z))
        return true;
    if(isinff(vec.x) || isinff(vec.y) || isinff(vec.z))
        return true;
    return false;
}

inline bool clAngleBetweenVectorsLessThan(const cl_float4& v1, const cl_float4& v2, const float degree, bool b_normalized)
{
    if(b_normalized)
        return (clDot(v1, v2) > cosf(DEG2RAD(degree)));
    else
        return (clDot(clNormalize(v1), clNormalize(v2)) > cosf(DEG2RAD(degree)));
}

inline bool clAngleBetweenVectorsLargerThan(const cl_float4& v1, const cl_float4& v2, const float degree, bool b_normalized)
{
    if(b_normalized)
        return (clDot(v1, v2) < cosf(DEG2RAD(degree)));
    else
        return (clDot(clNormalize(v1), clNormalize(v2)) < cosf(DEG2RAD(degree)));
}

inline float clNormalDistance(const cl_float4& normal, const cl_float4& point1, const cl_float4& point2)
{
    return fabsf(clDot(normal, point1 - point2));
}


#endif // CLOPERATORS_H
