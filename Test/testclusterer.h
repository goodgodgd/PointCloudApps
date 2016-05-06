#ifndef TESTCLUSTERER_H
#define TESTCLUSTERER_H

#include "PCWork/Clustering/objectclusterer.h"

inline QImage TestBorderLine(ObjectClusterer& objCluster, QImage& colorImg)
{
    static int lineCount=0;
    static int numBorders=0;
    const vecLines& lines = objCluster.borderLines;
    const cl_int* objectMap = objCluster.GetObjectMap();
    const vecSegment* objects = objCluster.GetObjects();
    QImage borderImg = colorImg;

    if(numBorders != lines.size() || lines.size() < ++lineCount)
    {
        numBorders = lines.size();
        lineCount = 0;
    }

    QRgb color;
    const Segment* leftPlane = objCluster.GetObjectByID(objCluster.IdPairs[lineCount].first);
    const Segment* rightPlane = objCluster.GetObjectByID(objCluster.IdPairs[lineCount].second);
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
             << lines[lineCount].endPixels[0] << lines[lineCount].endPixels[1];


    for(int y=leftPlane->rect.yl; y<=leftPlane->rect.yh; y++)
    {
        for(int x=leftPlane->rect.xl; x<=leftPlane->rect.xh; x++)
        {
            if(objectMap[IMGIDX(y,x)]!=leftPlane->id)
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
            if(objectMap[IMGIDX(y,x)]!=rightPlane->id)
                continue;
            color = colorImg.pixel(x,y);
            color = qRgb(qRed(color)/2, qGreen(color)/2, 255);
            borderImg.setPixel(x,y,color);
        }
    }

    QPoint pixel;
    PairOfPixels line(lines[lineCount].endPixels[0], lines[lineCount].endPixels[1]);
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
    borderImg.setPixel(pixel, qRgb(255,255,0));
    pixel = QPoint(objCluster.virutalPixels[lineCount*2+1].x, objCluster.virutalPixels[lineCount*2+1].y);
    borderImg.setPixel(pixel, qRgb(255,255,0));

    return borderImg;
}

#endif // TESTCLUSTERER_H
