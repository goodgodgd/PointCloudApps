#ifndef VIRTUALSPHERE_H
#define VIRTUALSPHERE_H

#include "ivirtualshape.h"
#include "IO/VirtualSensor/readerutil.h"

class VirtualSphere : public IVirtualShape
{
    float radius;

public:
    enum Enum
    {
        NUM_ATTRIB = 4
    };

    VirtualSphere(MapQStrFloat& attribMap)
    {
        type = SPHERE;
        QStringList attribList;
        attribList << "center_x" << "center_y" << "center_z" << "radius";
        assert(attribList.size()==NUM_ATTRIB);
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        center.x = attribMap[attribList[0]];
        center.y = attribMap[attribList[1]];
        center.z = attribMap[attribList[2]];
        center.w = 0.f;
        radius = attribMap[attribList[3]];
        qDebug() << "load sphere" << center << radius;
    }

    virtual ~VirtualSphere() {}

    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersectOut) const
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
        float sqAdd = b*b - a*c;
        if(sqAdd < 0)
            return false;

        float t1 = (-b + sqrt(sqAdd))/a;
        float t2 = (-b - sqrt(sqAdd))/a;
        if(t1 < 0 || t2 < 0)
            return false;

        float t = smin(t1, t2);

        intersectOut = campos + raydir*t;
        assert(fabsf(clLength(intersectOut - center) - radius) < 0.001f);
    }
};


#endif // VIRTUALSPHERE_H
