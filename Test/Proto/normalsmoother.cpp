#include "normalsmoother.h"

TestNormalSmoother* NormalSmoother::tester = nullptr;

NormalSmoother::NormalSmoother()
{
}

void NormalSmoother::SmootheNormalCloud(cl_float4* pointCloud, cl_float4* normalCloud)
{
    static cl_float4* normalBuffer = new cl_float4[IMAGE_WIDTH*IMAGE_HEIGHT];
    cl_float4 adjacentNormals[ADJ_SIZE];
    cl_int2 pixel;
    int imgidx;

    for(int y=0+MG; y<IMAGE_HEIGHT-MG; y++)
    {
        for(int x=0+MG; x<IMAGE_WIDTH-MG; x++)
        {
            pixel = (cl_int2){x, y};
            imgidx = IMGIDX(y,x);

            int numNormals = GetAdjacentNormals(pointCloud, normalCloud, pixel, adjacentNormals);
            if(numNormals > ADJ_SIZE/2)
                normalBuffer[imgidx] = AverageInlierNormals(adjacentNormals, numNormals);
            else
                normalBuffer[imgidx] = normalCloud[imgidx];

            // Test here
//            if(tester != nullptr)
//                tester->CheckNormalSmoothed(pointCloud, normalCloud, normalBuffer);
        }
    }

    memcpy(normalCloud, normalBuffer, IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4));
}

int NormalSmoother::GetAdjacentNormals(cl_float4* pointCloud, cl_float4* normalCloud, cl_int2 pixel, cl_float4* adjacentNormals)
{
    // addpt.x, y <= MG to ensure not using out-of-image index
    static const cl_int2 addpt[] = {{0,0}, {1,2}, {-1,2}, {1,-2}, {-1,-2}, {2,0}, {-2,0}};
    cl_int2 adjPoint;
    int adjIndex;
    int numAdj = 0;
    const float centerDepth = DEPTH(pointCloud[IMGIDX(pixel.y, pixel.x)]);
    const float depthDiffLimit = centerDepth*0.05;

    for(int i=0; i<ADJ_SIZE; i++)
    {
        adjPoint = pixel + addpt[i];
        adjIndex = IMGIDX(adjPoint.y, adjPoint.x);
        if(clIsNull(normalCloud[adjIndex]))
            continue;
        if((DEPTH(pointCloud[adjIndex]) - centerDepth) < depthDiffLimit)
            continue;

        adjacentNormals[numAdj++] = normalCloud[adjIndex];
    }
    return numAdj;
}

cl_float4 NormalSmoother::AverageInlierNormals(cl_float4* adjacentNormals, const int numNormals)
{
    const cl_float4& hereNormal = adjacentNormals[0];
    cl_float4 avgNormal = AverageNormals(adjacentNormals, numNormals);

    if(AngleBetweenVectorsLargerThan(hereNormal, avgNormal, 5.f, true))
        return hereNormal;

    // sort normal vectors w.r.t distance to avgNormal
    IdxVal<float> dots[ADJ_SIZE];
    for(int i=0; i<numNormals; i++)
    {
        dots[i].idx = i;
        dots[i].val = clDot(avgNormal, adjacentNormals[i]);
    }
    BubbleSortDescending(dots, numNormals);

    // check index of idx=0


    // search outlier
    const int mididx = numNormals/2;
    // equation of trigonometric functions
    // cos(2a) = 2cos^2(a) - 1
    const float dotLowLimit = 2.f*dots[mididx].val*dots[mididx].val - 1.f;
    int inlierUntil;
    // angle between inlier vector and average vector < twice of angle between median vector and average vector
    for(inlierUntil=mididx+1; inlierUntil<numNormals; inlierUntil++)
        if(dots[inlierUntil].val > dotLowLimit)
            break;

    if(inlierUntil==numNormals)
        return hereNormal;

    return AverageNormals(adjacentNormals, inlierUntil);
}

cl_float4 NormalSmoother::AverageNormals(cl_float4* normals, const int num)
{
    cl_float4 avg = (cl_float4){0,0,0,0};
    for(int i=0; i<num; i++)
        avg = avg + normals[i];
    return clNormalize(avg);
}

bool NormalSmoother::AngleBetweenVectorsLargerThan(const cl_float4& v1, const cl_float4& v2, const float degree, bool b_normalized)
{
    if(b_normalized)
        return (clDot(v1, v2) > cosf(DEG2RAD(degree))) ? true : false;
    else
        return (clDot(clNormalize(v1), clNormalize(v2)) > cosf(DEG2RAD(degree))) ? true : false;
}

void NormalSmoother::SetTester(TestNormalSmoother* tester_in)
{
    tester = tester_in;
}
