#ifndef TESTNORMALSMOOTHER_H
#define TESTNORMALSMOOTHER_H

#include "Share/project_common.h"
#include "Share/drawutils.h"

class TestNormalSmoother
{
public:
    TestNormalSmoother()
    {

    }

    void CheckNormalSmoothed(cl_float4* pointCloud, cl_float4* normalCloud, cl_float4& normalAfter
                             , cl_int2& centerPixel, cl_int2* adjacentPixels, const int numInlier, const int numAdj)
    {
        const cl_float4& centerPoint = pointCloud[IMGIDX(centerPixel.y, centerPixel.x)];
        const cl_float4& centerNormal = normalCloud[IMGIDX(centerPixel.y, centerPixel.x)];
        int imgidx;
        if(clDot(centerNormal, normalAfter) > cosf(DEG2RAD(30.f)))
            return;
        if(centerPixel.x%5>0)
            return;

        // draw inliers
        for(int i=0; i<numInlier; i++)
        {
            imgidx = IMGIDX(adjacentPixels[i].y, adjacentPixels[i].x);
            DrawUtils::MarkPoint3D(pointCloud[imgidx], normalCloud[imgidx], qRgb(0,255,0), 0.05f);
        }
        // draw outliers
        for(int i=numInlier; i<numAdj; i++)
        {
            imgidx = IMGIDX(adjacentPixels[i].y, adjacentPixels[i].x);
            DrawUtils::MarkPoint3D(pointCloud[imgidx], normalCloud[imgidx], qRgb(100,100,100), 0.05f);
        }

        // draw befor normal
        DrawUtils::MarkPoint3D(centerPoint, centerNormal, qRgb(0,0,255), 0.1f);

        // draw after normal
        DrawUtils::MarkPoint3D(centerPoint, normalAfter, qRgb(255,0,0), 0.1f);
        qDebug() << "testnormalsmoothe:" << centerPoint << centerNormal << adjacentPixels[1] << numInlier << numAdj << gvm::LnNum();
    }

private:

};

#endif // TESTNORMALSMOOTHER_H
