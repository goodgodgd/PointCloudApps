#include "normalsmoother.h"

NormalSmoother::NormalSmoother()
    : pointCloud(nullptr),
      normalCloud(nullptr)
{
}

void NormalSmoother::SmootheNormalCloud(cl_float4* pointCloud_in, cl_float4* normalCloud_in)
{
    static cl_float4* normalBuffer = new cl_float4[IMAGE_WIDTH*IMAGE_HEIGHT];
    cl_int2 adjacentPixels[ADJ_SIZE];
    cl_int2 pixel;
    int imgidx;

    pointCloud = pointCloud_in;
    normalCloud = normalCloud_in;

    for(int y=0+MG; y<IMAGE_HEIGHT-MG; y++)
    {
        for(int x=0+MG; x<IMAGE_WIDTH-MG; x++)
        {
            pixel = (cl_int2){x, y};
            imgidx = IMGIDX(y,x);

            if(clIsNull(normalCloud[imgidx]))
                continue;

            int numAdj = GetAdjacentPixels(pixel, adjacentPixels);
            if(numAdj < ADJ_SIZE/2)
            {
                normalBuffer[imgidx] = normalCloud[imgidx];
                continue;
            }

            int numInlier = ExtractInliers(pixel, adjacentPixels, numAdj);
            if(numInlier < ADJ_SIZE/2)
            {
                normalBuffer[imgidx] = normalCloud[imgidx];
                continue;
            }

            normalBuffer[imgidx] = AverageNormals(adjacentPixels, numInlier);
        }
    }
//    DrawUtils::DrawNormalCloud(pointCloud, normalCloud, (cl_float4){1.f,1.f,1.f,1.f});

    memcpy(normalCloud, normalBuffer, IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4));
}

int NormalSmoother::GetAdjacentPixels(cl_int2 centerPixel, cl_int2* adjacentPixels)
{
    // addpt.x, y <= MG to ensure not using out-of-image index
    static const cl_int2 addpt[] = {{0,0}, {1,2}, {-1,2}, {1,-2}, {-1,-2}, {2,0}, {-2,0}};
    cl_int2 adjPixel;
    int adjIndex;
    int numAdj = 0;
    const float centerDepth = DEPTH(pointCloud[IMGIDX(centerPixel.y, centerPixel.x)]);
    const float depthDiffLimit = centerDepth*0.05;

    for(int i=0; i<ADJ_SIZE; i++)
    {
        adjPixel = centerPixel + addpt[i];
        adjIndex = IMGIDX(adjPixel.y, adjPixel.x);
        if(clIsNull(normalCloud[adjIndex]))
            continue;
        if(fabsf(DEPTH(pointCloud[adjIndex]) - centerDepth) > depthDiffLimit)
            continue;

        adjacentPixels[numAdj++] = adjPixel;
    }
    return numAdj;
}

int NormalSmoother::ExtractInliers(cl_int2 centerPixel, cl_int2* adjacentPixels, const int numAdj)
{
    const cl_float4& hereNormal = normalCloud[IMGIDX(centerPixel.y, centerPixel.x)];
    cl_float4 avgNormal = AverageNormals(adjacentPixels, numAdj);

    // when working normal vector is close to average of neighbors, just use original normal vecotr
    if(AngleBetweenVectorsLessThan(hereNormal, avgNormal, 5.f, true))
        return 0;

    // compute average dot
    float neibDots[ADJ_SIZE];
    float avgDot = 0.f;
    for(int i=0; i<numAdj; i++)
    {
        const cl_float4& neibNormal = normalCloud[IMGIDX(adjacentPixels[i].y, adjacentPixels[i].x)];
        neibDots[i] = clDot(avgNormal, neibNormal);
        avgDot += neibDots[i];
    }
    avgDot /= (float)(numAdj);

    const float ratio = 0.7f;
    // cos(2a) = 2cos^2(a) - 1
    const float dotLowLimit = avgDot*ratio + (2.f*avgDot*avgDot - 1.f)*(1.f-ratio);
    int cntInlier = 0;

    // reject outlier pixels in adjacent pixels
    for(int i=0; i<numAdj; i++)
    {
        // if angle between avgNormal and adjacentNormal is small, it is inlier
        if(neibDots[i] > dotLowLimit)
        {
            if(cntInlier < i)
                Swap(adjacentPixels[cntInlier], adjacentPixels[i]);
            cntInlier++;
        }
    }

    return cntInlier;
}

cl_float4 NormalSmoother::AverageNormals(cl_int2* pixels, const int num)
{
    cl_float4 avg = (cl_float4){0,0,0,0};
    for(int i=0; i<num; i++)
        avg = avg + normalCloud[IMGIDX(pixels[i].y, pixels[i].x)];
    return clNormalize(avg);
}

bool NormalSmoother::AngleBetweenVectorsLessThan(const cl_float4& v1, const cl_float4& v2, const float degree, bool b_normalized)
{
    if(b_normalized)
        return (clDot(v1, v2) > cosf(DEG2RAD(degree)));
    else
        return (clDot(clNormalize(v1), clNormalize(v2)) > cosf(DEG2RAD(degree)));
}

void NormalSmoother::Swap(cl_int2& foo, cl_int2& bar)
{
    cl_int2 tmp;
    tmp = foo;
    foo = bar;
    bar = tmp;
}
