#ifndef VIRTUALSPHERE_H
#define VIRTUALSPHERE_H

#include "ivirtualshape.h"
#include "IO/VirtualSensor/readerutil.h"

class VirtualSphere : public IVirtualShape
{
    cl_float radius;

public:
    VirtualSphere(MapNameData& attribMap)
    {
        type = SPHERE;
        QStringList attribList;
        attribList << "center" << "radius";
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        center = attribMap[attribList[0]].GetVector();
        radius = attribMap[attribList[1]].GetValue();
        qDebug() << "load sphere" << center << radius;
    }

    virtual ~VirtualSphere() {}

    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersect)
    {
        // raydir must be normalized
        assert(fabsf(clSqLength(raydir)-1.f) < 0.001f);
        // line euqation: p = o + t*d
        // sphere equation: |p - c|^2 = r^2
        // intersecting point: |t'*d + (o-c)|^2 = r^2
        //                      t'^2*|d|^2 + 2*t'*dot(d,o-c) + |o-c|^2 - r^2 = 0;
        float a = clSqLength(raydir);
        float b = clDot(raydir, campos - center);
        float c = clSqLength(campos - center) - radius*radius;
        float determinant = b*b - a*c;
        if(determinant < 0)
            return false;

        float t1 = (-b + sqrt(determinant))/a;
        float t2 = (-b - sqrt(determinant))/a;
        float t = smin(t1, t2);
        if(t<=0) // intersect must be in positive ray direction
            return false;

        intersect = campos + raydir*t;
        assert(fabsf(clLength(intersect - center) - radius) < 0.001f);
        return true;
    }
};


#endif // VIRTUALSPHERE_H
