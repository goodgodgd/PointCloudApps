#include "pointsmoother.h"

#define MG      2

PointSmoother::PointSmoother()
{
    const int a=1;
    int b1 = a;
//    int& b2= a;   // compile error
    int c = 1;
    const int d1 = c;
    const int& d2 = c;
}

void PointSmoother::SmoothePointCloud(cl_float4* pointCloud, cl_float4* normalCloud)
{
    static cl_float4* pointBuffer = new cl_float4[IMAGE_WIDTH*IMAGE_HEIGHT];

    for(int y=0+MG; y<IMAGE_HEIGHT-MG; y++)
        for(int x=0+MG; x<IMAGE_WIDTH-MG; x++)
            pointBuffer[IMGIDX(y,x)] = SmoothePoint(pointCloud, (cl_int2){x, y}, normalCloud[IMGIDX(y,x)]);

    memcpy(pointCloud, pointBuffer, IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4));
}

cl_float4 PointSmoother::SmoothePoint(cl_float4* pointCloud, const cl_int2& pixel, cl_float4& normal)
{
    // find only linear direction
    const cl_int2 linearDir = SearchLinearDirection(pointCloud, pixel, normal);
    if(linearDir.x==0 && linearDir.y==0)
        return pointCloud[PIXIDX(pixel)];

    // smoothe and debug
    return SmoothePointByMeanDepth(pointCloud, pixel, linearDir);
}

cl_int2 PointSmoother::SearchLinearDirection(cl_float4* pointCloud, const cl_int2& pixel, cl_float4& normal)
{
    // direc.x, y <= MG
    static const cl_int2 direc[] = {{2,0}, {2,2}, {0,2}, {-2,2}};
    const int szdir = 4;
    const cl_float4& herept = pointCloud[PIXIDX(pixel)];
    const float hereNormalDepth = clDot(normal, herept);
    const float distUppLimit = hereNormalDepth*0.002f;
    float flatness, minDist = distUppLimit;
    int minIndex = -1;

    // search flattest direction
    for(int i=0; i<szdir; i++)
    {
        const cl_float4& leftpt = pointCloud[IMGIDX(pixel.y-direc[i].y, pixel.x-direc[i].x)];
        const cl_float4& righpt = pointCloud[IMGIDX(pixel.y+direc[i].y, pixel.x+direc[i].x)];
        if(clIsNull(leftpt) || clIsNull(righpt))
            continue;

        flatness = clDot(normal, (leftpt - righpt));
        if(flatness < minDist)
        {
            minDist = flatness;
            minIndex = i;
        }
    }

    if(minIndex < 0)
        return (cl_int2){0,0};
    else
        return direc[minIndex];
}

cl_float4 PointSmoother::SmoothePointByMeanDepth(cl_float4* pointCloud, const cl_int2& pixel, const cl_int2& linearDir)
{
    const cl_float4& herept = pointCloud[PIXIDX(pixel)];
    const cl_float4& righpt = pointCloud[IMGIDX(pixel.y+linearDir.y, pixel.x+linearDir.x)];
    const cl_float4& leftpt = pointCloud[IMGIDX(pixel.y-linearDir.y, pixel.x-linearDir.x)];

    float meanDepth = (DEPTH(herept) + DEPTH(leftpt) + DEPTH(righpt)) / 3.f;
    cl_float4 smoothedpt = herept / DEPTH(herept) * meanDepth;

    // Test:: normal distances of herept righpt leftpt smoothedpt

    return smoothedpt;
}






