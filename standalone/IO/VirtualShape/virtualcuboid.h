#ifndef VIRTUALCUBOID_H
#define VIRTUALCUBOID_H

#include "ivirtualshape.h"
#include "IO/VirtualSensor/readerutil.h"
#include "virtualrectplane.h"

class VirtualCuboid : public IVirtualShape
{
    std::vector<VirtualRectPlane> surfaces;
    cl_float4 dimension;

public:
    int totalCount;
    int outlierCount;

    VirtualCuboid(MapNameData& attribMap)
    {
        type = CUBOID;
        QStringList attribList;
        attribList << "center" << "dimension" << "vertical" << "horizontal";
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        cl_float4 axes[3];
        center = attribMap[attribList[0]].GetVector();
        dimension = attribMap[attribList[1]].GetVector();
        axes[0] = clNormalize(attribMap[attribList[2]].GetVector());    // vertical
        axes[1] = clNormalize(attribMap[attribList[3]].GetVector());    // horizontal
        axes[2] = clNormalize(clCross(axes[0], axes[1]));               // normal
        axes[1] = clNormalize(clCross(axes[2], axes[0]));

        surfaces.clear();
        for(int i=0; i<6; i++)
        {
            MapNameData attribRectMap = TranslateToRectAttrib(center, dimension, axes, i);
            surfaces.emplace_back(attribRectMap);
        }

        qDebug() << "load cuboid" << center << dimension;
    }

    MapNameData TranslateToRectAttrib(const cl_float4& center, const cl_float4& dimension, const cl_float4* axes, const int surfaceIndex)
    {
        static QStringList attribList;
        if(attribList.isEmpty())
            attribList << "center" << "vertical" << "horizontal" << "height" << "width";
        int dir = (surfaceIndex%2==0) ? 1 : -1;
        int axisIdx = surfaceIndex/2;

        cl_float4 rectCenter = center + axes[axisIdx] * dir*dimension.s[axisIdx]/2.f;
        cl_float4 vertical = axes[(axisIdx+1)%3];
        cl_float4 horizontal = axes[(axisIdx+2)%3];
        cl_float height = dimension.s[(axisIdx+1)%3];
        cl_float width = dimension.s[(axisIdx+2)%3];

        MapNameData attribRectMap;
        attribRectMap.insert(PairNameData(attribList[0], AttribData(rectCenter)));
        attribRectMap.insert(PairNameData(attribList[1], AttribData(vertical)));
        attribRectMap.insert(PairNameData(attribList[2], AttribData(horizontal)));
        attribRectMap.insert(PairNameData(attribList[3], AttribData(height)));
        attribRectMap.insert(PairNameData(attribList[4], AttribData(width)));

        return attribRectMap;
    }

    virtual ~VirtualCuboid() {}

    virtual bool IntersectingPointFromRay(const cl_float4& campos, const cl_float4& raydir, cl_float4& intersect)
    {
        // raydir must be normalized
        assert(fabsf(clSqLength(raydir)-1.f) < 0.001f);

        int touch=0;
        cl_float4 tmpPoint;
        float minDist = 10000.f;
        for(VirtualRectPlane& rect: surfaces)
        {
            if(rect.IntersectingPointFromRay(campos, raydir, tmpPoint))
            {
                if(minDist > clSqLength(campos - tmpPoint))
                {
                    minDist = clSqLength(campos - tmpPoint);
                    intersect = tmpPoint;
                }
                touch++;
            }
        }

        if(touch>0)
        {
            totalCount++;
            if(touch!=2)
                outlierCount++;
        }

        return (touch>0);
    }
};

#endif // VIRTUALCUBOID_H
