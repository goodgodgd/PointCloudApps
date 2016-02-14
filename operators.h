#ifndef OPERATORS_H
#define OPERATORS_H

#include "project_common.h"

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


#endif // OPERATORS_H
