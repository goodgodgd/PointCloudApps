#ifndef VIRTUALRECTPLANE_H
#define VIRTUALRECTPLANE_H

#include "ivirtualshape.h"
#include "IO/VirtualSensor/readerutil.h"

class VirtualRectPlane : public IVirtualShape
{

public:
    float width;
    float height;
    cl_float4 hordir;
    cl_float4 verdir;
    cl_float4 normal;

    enum Enum
    {
        NUM_ATTRIB = 5
    };

    VirtualRectPlane(MapNameData& attribMap)
    {
        type = RECT;
        QStringList attribList;
        attribList << "center" << "vertical" << "horizontal" << "height" << "width";
        assert(attribList.size()==NUM_ATTRIB);
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        center = attribMap[attribList[0]].GetVector();
        verdir = attribMap[attribList[1]].GetVector();
        hordir = attribMap[attribList[2]].GetVector();
        height = attribMap[attribList[3]].GetValue();
        width = attribMap[attribList[4]].GetValue();
        normal = clCross(verdir, hordir);
        hordir = clCross(normal, verdir);
        qDebug() << "load rect" << center << verdir << hordir << height << width;
    }

    virtual ~VirtualRectPlane() {}

    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersect)
    {
        const float equalMargin = 0.00001f;
        // raydir must be normalized
        assert(fabsf(clSqLength(raydir)-1.f) < equalMargin);
        // line euqation: p = o + t*r
        // plane equation: dot(p,n) = dot(c,n)
        // intersecting point: t' = dot(c-o,n)/dot(r,n)
        const float angleUpLimit = 80.f;
        if(clDot(raydir, normal) >= 0 && clAngleBetweenVectorsLargerThan(raydir, normal, angleUpLimit, true))
            return false;
        else if(clDot(raydir, -normal) > 0 && clAngleBetweenVectorsLargerThan(raydir, -normal, angleUpLimit, true))
            return false;

        float t = clDot(center - campos, normal) / clDot(raydir, normal);
        assert(!isnanf(t) && !isinff(t));
        if(t<0) // intersect must be in positive ray direction
            return false;

        intersect = campos + raydir*t;
        // intersect point must be on the plane
        assert(fabsf(clDot(campos - center, normal) - clDot(campos - intersect, normal)) < equalMargin);

        if(fabsf(clDot(intersect - center, verdir)) > height/2.f || fabsf(clDot(intersect - center, hordir)) > width/2.f)
            return false;
        return true;
    }
};

#endif // VIRTUALRECTPLANE_H
