#ifndef VIRTUALELLIPSOID_H
#define VIRTUALELLIPSOID_H

#include "ivirtualshape.h"
#include "IO/VirtualSensor/readerutil.h"

class VirtualEllipsoid : public IVirtualShape
{
    cl_float4 axes[3];
    cl_float4 dimension;

public:
    VirtualEllipsoid(MapNameData& attribMap)
    {
        type = ELLIPSOID;
        QStringList attribList;
        attribList << "center" << "axis1" << "axis2" << "dimension";
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        center = attribMap[attribList[0]].GetVector();
        axes[0] = clNormalize(attribMap[attribList[1]].GetVector());    // vertical
        axes[1] = clNormalize(attribMap[attribList[2]].GetVector());    // horizontal
        axes[2] = clNormalize(clCross(axes[0], axes[1]));               // normal
        axes[1] = clNormalize(clCross(axes[2], axes[0]));
        dimension = attribMap[attribList[3]].GetVector();
        qDebug() << "load ellipsoid" << center << axes[0] << axes[1] << dimension;
    }

    virtual ~VirtualEllipsoid() {}

    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersect)
    {
        // raydir must be normalized
        assert(fabsf(clSqLength(raydir)-1.f) < 0.001f);
        // line euqation: p = o + t*d
        // ellipse equation: (p-c)'*A*(p-c) = 1 -> (t*d + s)'*A*(t*d + s) = 1   where s=o-c
        //                  where A = 1/m[0]^2 * r[0]*r[0]' + 1/m[1]^2 * r[1]*r[1]' + 1/m[2]^2 * r[2]*r[2]'
        //                  where m = dimension, r = axes
        // intersecting point: t^2*d'*A*d + 2t*d'*A*s + s'*A*s - 1 = 0;
        float dAd=0, dAs=0, sAs=0;
        cl_float4 relCenter = campos - center;  // ==s
        for(int i=0; i<3; i++)
        {
            dAd += 1.f/powf(dimension.s[i], 2.f) * powf(clDot(raydir, axes[i]), 2.f);
            dAs += 1.f/powf(dimension.s[i], 2.f) * clDot(raydir, axes[i]) * clDot(axes[i], relCenter);
            sAs += 1.f/powf(dimension.s[i], 2.f) * powf(clDot(relCenter, axes[i]), 2.f);
        }

        float t = ReaderUtil::QuadraticSolver(dAd, dAs, sAs-1.f);
        if(t<=0) // intersect must be in positive ray direction
            return false;

        intersect = campos + raydir*t;

        float chkVal=0;
        for(int i=0; i<3; i++)
            chkVal += powf(clDot(intersect - center, axes[i]), 2.f) / powf(dimension.s[i], 2.f);
        assert(fabsf(chkVal - 1.f) < 0.001f);
        return true;
    }
};

#endif // VIRTUALELLIPSOID_H
