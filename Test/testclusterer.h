#ifndef TESTCLUSTERER_H
#define TESTCLUSTERER_H

#include "PCWork/Clustering/objectclusterer.h"

inline QImage CheckObjectCluster(ObjectClusterer& objCluster, QImage& colorImg)
{
    static int lineCount=0;
    static int numBorders=0;
#ifdef RESERVE_DEBUG_INFO
    if(numBorders != objCluster.IdPairs.size() || objCluster.IdPairs.size() < ++lineCount)
    {
        numBorders = objCluster.IdPairs.size();
        lineCount = 0;
    }

    const ImLine& borderLine = objCluster.borderLines[lineCount];
    const cl_int* planeMap = objCluster.srcPlaneMap;
    QImage borderImg = colorImg;
    const cl_float4& borderPoint = objCluster.borderPoints[lineCount];
    const PairOfPoints& pointPair = objCluster.pointPairs[lineCount];
    const PairOfFloats& heightPair = objCluster.heights[lineCount];

    QRgb color;
    const Segment* leftPlane = objCluster.GetPlaneByID(objCluster.IdPairs[lineCount].first);
    const Segment* rightPlane = objCluster.GetPlaneByID(objCluster.IdPairs[lineCount].second);
    if(leftPlane==nullptr || rightPlane==nullptr)
    {
        qDebug() << "PlanePair Error: null" << lineCount << objCluster.IdPairs[lineCount].first << objCluster.IdPairs[lineCount].second;
        return borderImg;
    }
    if(leftPlane->id<0 || rightPlane->id<0)
    {
        qDebug() << "PlanePair Error!" << leftPlane->id << rightPlane->id;
        return borderImg;
    }
    qDebug() << "PlanePair" << lineCount << leftPlane->id << rightPlane->id << "numpt" << leftPlane->numpt << rightPlane->numpt
                << borderLine.endPixels[0] << borderLine.endPixels[1] << "angle" << objCluster.betweenAngles[lineCount]
                    << "heights" << heightPair.first << heightPair.second << (heightPair.first > heightPair.second);
    qDebug() << "   points" << borderPoint << pointPair.first << pointPair.second;


    // dye leftPlane
    for(int y=leftPlane->rect.yl; y<=leftPlane->rect.yh; y++)
    {
        for(int x=leftPlane->rect.xl; x<=leftPlane->rect.xh; x++)
        {
            if(planeMap[IMGIDX(y,x)]!=leftPlane->id)
                continue;
            color = colorImg.pixel(x,y);
            color = qRgb(255, qGreen(color)/2, qBlue(color)/2);
            borderImg.setPixel(x,y,color);
        }
    }
    for(int y=rightPlane->rect.yl; y<=rightPlane->rect.yh; y++)
    {
        for(int x=rightPlane->rect.xl; x<=rightPlane->rect.xh; x++)
        {
            if(planeMap[IMGIDX(y,x)]!=rightPlane->id)
                continue;
            color = colorImg.pixel(x,y);
            color = qRgb(qRed(color)/2, qGreen(color)/2, 255);
            borderImg.setPixel(x,y,color);
        }
    }

    QPoint pixel;
    PairOfPixels line(borderLine.endPixels[0], borderLine.endPixels[1]);
    if(abs(line.second.x - line.first.x) > abs(line.second.y - line.first.y))
    {
        cl_int2 begpx = ((line.first.x < line.second.x) ? line.first : line.second);
        cl_int2 endpx = ((line.first.x < line.second.x) ? line.second : line.first);
        float grad = (float)(endpx.y - begpx.y)/(float)(endpx.x - begpx.x);
        for(int x=begpx.x; x<=endpx.x; x++)
        {
            pixel = QPoint(x, begpx.y+(int)(grad*(float)(x-begpx.x)));
            borderImg.setPixel(pixel, qRgb(0,255,0));
        }
    }
    else
    {
        cl_int2 begpx = ((line.first.y < line.second.y) ? line.first : line.second);
        cl_int2 endpx = ((line.first.y < line.second.y) ? line.second : line.first);
        float grad = (float)(endpx.x - begpx.x)/(float)(endpx.y - begpx.y);
        for(int y=begpx.y; y<=endpx.y; y++)
        {
            pixel = QPoint(begpx.x+(int)(grad*(float)(y-begpx.y)), y);
            borderImg.setPixel(pixel, qRgb(0,255,0));
        }
    }

    pixel = QPoint(objCluster.virutalPixels[lineCount*2].x, objCluster.virutalPixels[lineCount*2].y);
    borderImg.setPixel(pixel, qRgb(0,0,255));
    pixel = QPoint(objCluster.virutalPixels[lineCount*2+1].x, objCluster.virutalPixels[lineCount*2+1].y);
    borderImg.setPixel(pixel, qRgb(255,0,0));

    return borderImg;
#else
    return colorImg;
#endif
}

#endif // TESTCLUSTERER_H
