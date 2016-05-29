#ifndef VIRTUALCYLINDER_H
#define VIRTUALCYLINDER_H

#include "ivirtualshape.h"
#include "IO/VirtualSensor/readerutil.h"

class VirtualCylinder : public IVirtualShape
{
    cl_float4 axis;
    cl_float radius;
    cl_float height;

public:
    VirtualCylinder(MapNameData& attribMap)
    {
        type = CYLINDER;
        QStringList attribList;
        attribList << "center" << "axis" << "radius" << "height";
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        center = attribMap[attribList[0]].GetVector();
        axis = clNormalize(attribMap[attribList[1]].GetVector());
        radius = attribMap[attribList[2]].GetValue();
        height = attribMap[attribList[3]].GetValue();
        qDebug() << "load cylinder" << center << axis << radius << height;
    }

    virtual ~VirtualCylinder() {}

    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersect)
    {
        // raydir must be normalized
        assert(fabsf(clSqLength(raydir)-1.f) < 0.001f);
        // line euqation: p = o + t*d
        // cylinder equation: |p-c|^2 - (dot(a, p-c))^2 = r^2
        // intersecting point: |t*d + s|^2 - (dot(a, t*d + s))^2 = r^2  where s=o-c
        //                      t^2*(|d|^2 - dot(a,d)^2) + 2t*(dot(d,s) - dot(a,d)*dot(a,s)) + |s|^2 - dot(a,s)^2 - |r|^2 = 0
        float ad = clDot(axis, raydir);
        float ds = clDot(raydir, campos - center);
        float as = clDot(axis, campos - center);
        float t = ReaderUtil::QuadraticSolver(clSqLength(raydir) - ad*ad
                                              , ds - ad*as
                                              , clSqLength(campos-center) - as*as - radius*radius);
        if(t<=0) // intersect must be in positive ray direction
            return false;

        intersect = campos + raydir*t;
        assert(fabsf(clSqLength(intersect - center) - powf(clDot(intersect - center, axis), 2.f) - radius*radius) < 0.0001f);

        return (fabsf(clDot(intersect - center, axis)) < height);
    }
};


#endif // VIRTUALCYLINDER_H
