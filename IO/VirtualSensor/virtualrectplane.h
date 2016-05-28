#ifndef VIRTUALRECTPLANE_H
#define VIRTUALRECTPLANE_H

#include "ivirtualshape.h"
#include "readerbase.h"

class VirtualRectPlane : public IVirtualShape
{
    float width;
    float height;

public:
    enum Enum
    {
        NUM_ATTRIB = 5
    };

    VirtualRectPlane(MapQStrFloat& attribMap)
    {
        type = RECT;
        QStringList attribList;
        attribList << "center_x" << "center_y" << "center_z" << "width" << "height";
        assert(attribList.size()==NUM_ATTRIB);
        ReaderBase::CheckIntegrity(attribMap, attribList);

        center.x = attribMap[attribList[0]];
        center.y = attribMap[attribList[1]];
        center.z = attribMap[attribList[2]];
        center.w = 0.f;
        width = attribMap[attribList[3]];
        height = attribMap[attribList[4]];
        qDebug() << "load rect" << center << width << height;
    }

    virtual ~VirtualRectPlane() {}

    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersect) const
    {
        // raydir must be normalized
        assert(fabsf(clSqLength(raydir)-1.f) < 0.001f);
        // line euqation: p = o + t*r
        // plane equation: dot(p,n) = dot(c,n)
        // intersecting point: t' = dot(c-o,n)/dot(r,n)
        static const cl_float4 planeNormal = (cl_float4){0,0,1,0};
        const float angleUpLimit = 80.f;
        if(clAngleBetweenVectorsLargerThan(raydir, -planeNormal, angleUpLimit, true))
            return false;

//        if(fabsf(raydir.x-0.7071f)<0.0001f && fabsf(raydir.y)<0.0001f && fabsf(raydir.z+0.7071f)<0.0001f)
//            qDebug() << "Intersect1" << raydir;

        float t = clDot(center - campos, planeNormal) / clDot(raydir, planeNormal);
        intersect = campos + raydir*t;
        // intersect point must be on the plane
        assert(fabsf(intersect.z-center.z) < 0.001f);

        if(intersect.x < center.x - height/2.f || intersect.x > center.x + height/2.f)
            return false;
        if(intersect.y < center.y - width/2.f || intersect.y > center.y + width/2.f)
            return false;
        return true;
    }
};


#endif // VIRTUALRECTPLANE_H
