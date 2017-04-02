#ifndef CLOPERATORS_H
#define CLOPERATORS_H

#include <QImage>
#include "Share/project_common.h"

inline cl_float4 clNormalize(const cl_float4& src);
inline float clDot(const cl_float4& v1, const cl_float4& v2);
inline bool clIsNull(const cl_float4& src, const float precision=0.0001f);
inline bool clIsNull(const cl_float8& src, const float precision=0.0001f);

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

inline cl_int2 operator /(const cl_int2& src, const int val)
{
    cl_int2 dst;
    dst.x = src.x / val;
    dst.y = src.y / val;
    return dst;
}

inline cl_float4 operator-(const cl_float4& src)
{
   return src*(-1.f);
}

inline cl_float8 operator *(const cl_float8& src, const float val)
{
    cl_float8 dst;
    dst.s[0] = src.s[0] * val;
    dst.s[1] = src.s[1] * val;
    dst.s[2] = src.s[2] * val;
    dst.s[3] = src.s[3] * val;
    dst.s[4] = src.s[4] * val;
    dst.s[5] = src.s[5] * val;
    dst.s[6] = src.s[6] * val;
    dst.s[7] = src.s[7] * val;
    return dst;
}

inline cl_float8 operator-(const cl_float8& src)
{
   return src*(-1.f);
}

inline cl_float8 operator -(const cl_float8& v1, const cl_float8& v2)
{
    cl_float8 dst;
    for(int i=0; i<8; ++i)
        dst.s[i] = v1.s[i] - v2.s[i];
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

inline QDebug operator <<(QDebug debug, const cl_float8 &c)
{
    QDebugStateSaver saver(debug);
    debug.space() << "clf8(" << c.s[0] << c.s[1] << c.s[2] << c.s[3] << c.s[4] << c.s[5] << c.s[6] << c.s[7] << ')';
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

inline QDebug operator <<(QDebug debug, const cl_uint2 &c)
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

inline bool clIsNull(const cl_float8& src, const float precision)
{
    for(int i=0; i<8; i++)
        if(fabsf(src.s[i]) > precision)
            return false;
    return true;
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

inline float clAngleBetweenVectorsDegree(const cl_float4& v1, const cl_float4& v2, bool b_normalized)
{
    float angle;
    if(b_normalized)
        angle = acosf(clDot(v1, v2));
    else
        angle = acosf(clDot(clNormalize(v1), clNormalize(v2)));
    return RAD2DEG(angle);
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
    cl_float4 ptdiff = point1 - point2;
    return fabsf(clDot(normal, ptdiff));
}

inline bool clIsNormalized(const cl_float4& vec)
{
    if(fabsf(clSqLength(vec) - 1.f) < 0.0001f)
        return true;
    else
        return false;
}

inline void clSplit(const cl_float8& src, cl_float4& dst1, cl_float4& dst2)
{
    dst1 = (cl_float4){src.s[0], src.s[1], src.s[2], src.s[3]};
    dst2 = (cl_float4){src.s[4], src.s[5], src.s[6], src.s[7]};
}

#endif // CLOPERATORS_H
