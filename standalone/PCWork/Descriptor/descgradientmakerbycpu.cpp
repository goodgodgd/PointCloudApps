#include "descgradientmakerbycpu.h"

int dbgx, dbgy, dbgidx, dbgcnt;

DescGradientMakerByCpu::DescGradientMakerByCpu()
{
    descriptorArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
}

void DescGradientMakerByCpu::CopmuteGradient(const cl_float4* pointCloud, const DescType* curvatures, AxesType* descAxes
                                             , const cl_int* neighborIndices, const cl_int* numNeighbors
                                             , const int maxNeighbs, const float descRadius)
{
    DescType* descriptors = descriptorArray.GetArrayPtr();
    memcpy(descriptors, curvatures, descriptorArray.ByteSize());

    dbgcnt = 0;
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            dbgx = x; dbgy = y; dbgidx = IMGIDX(y,x);
            try {
                CopmuteEachGradient(IMGIDX(y,x), pointCloud, curvatures, descAxes
                                    , neighborIndices, numNeighbors, maxNeighbs, descRadius);
            }
            catch (GradientException exception) {
                qDebug() << "GradientExcpetion:" << exception.msg_;
                descriptors[IMGIDX(y,x)].s[2] = 0.f;
                descriptors[IMGIDX(y,x)].s[3] = 0.f;
            }
        }
    }
}

void DescGradientMakerByCpu::CopmuteEachGradient(const int ctidx, const cl_float4* pointCloud
                                                 , const DescType* curvatures, AxesType* descAxes
                                                 , const cl_int* neighborIndices, const cl_int* numNeighbors
                                                 , const int maxNeighbs, const float descRadius)
{
    DescType* descriptors = descriptorArray.GetArrayPtr();
    if(numNeighbors[ctidx] < maxNeighbs/2)
        return;
    const cl_float4 thispoint = pointCloud[ctidx];
    const AxesType thisaxes = descAxes[ctidx];
    const int niOffset = ctidx * maxNeighbs;
    dbgcnt++;

    descriptors[ctidx].s[2] = DirectedGradient(pointCloud, curvatures, thispoint, thisaxes, true
                                               , neighborIndices, niOffset, numNeighbors[ctidx], descRadius);

    descriptors[ctidx].s[3] = DirectedGradient(pointCloud, curvatures, thispoint, thisaxes, false
                                               , neighborIndices, niOffset, numNeighbors[ctidx], descRadius);

    if(descriptors[ctidx].s[2] < 0.f)
        descAxes[ctidx] = -descAxes[ctidx];
}

float DescGradientMakerByCpu::DirectedGradient(const cl_float4* pointCloud, const DescType* curvatures
                                               , const cl_float4 thispoint, const cl_float8 thisaxes, const bool majorAxis
                                               , const cl_int* neighborIndices, const int niOffset
                                               , const int numNeighb, const float descRadius)
{
    const cl_float4 curvdir = (majorAxis) ? (cl_float4){thisaxes.s[0], thisaxes.s[1], thisaxes.s[2], 0}
                                                : (cl_float4){thisaxes.s[4], thisaxes.s[5], thisaxes.s[6], 0};
    const cl_float4 orthdir = (!majorAxis)? (cl_float4){thisaxes.s[0], thisaxes.s[1], thisaxes.s[2], 0}
                                                : (cl_float4){thisaxes.s[4], thisaxes.s[5], thisaxes.s[6], 0};

    float A[MAX_NEIGHBORS*2];
    float b[MAX_NEIGHBORS];
    cl_float4 diff;
    int nbidx, vcount=0;

    // set A and b in Ax=b
    for(int i=0; i<numNeighb; i++)
    {
        nbidx = neighborIndices[niOffset + i];
        diff = pointCloud[nbidx] - thispoint;
        if(fabsf(clDot(orthdir, diff)) > descRadius/2.f)
            continue;
        diff = diff * DESC_SCALE;
        A[i*2] = clDot(curvdir, diff);
        A[i*2+1] = 1.f;
        b[i] = (majorAxis) ? curvatures[nbidx].x : curvatures[nbidx].y;
        vcount++;
        if(vcount>=MAX_NEIGHBORS)
            break;
    }

//    if(dbgcnt < 100)
//    {
//        qDebug() << "dirgrad1" << dbgcnt << dbgx << dbgy << vcount << curvdir;
//        for(int d=0; d<vcount; d++)
//            if(isnanf(A[d*2+0]) || isnanf(A[d*2+1]))
//                throw GradientException("nan A");
//    }

    // A'*A
    float AtA[4];
    for(int i=0; i<2; i++)
    {
        for(int k=0; k<2; k++)
        {
            AtA[i*2 + k] = 0.f; // A.col(i) * A.col(k)
            for(int d=0; d<vcount; d++)
                AtA[i*2 + k] += A[d*2 + i]*A[d*2 + k];
        }
    }

    // inv(A'*A)
    float AtAi[4];
    float det = AtA[0]*AtA[3] - AtA[1]*AtA[2];
    if(fabsf(det) < 0.001f)
        throw GradientException("zero determinant");
    AtAi[0] = AtA[3]/det;
    AtAi[3] = AtA[0]/det;
    AtAi[1] = -AtA[1]/det;
    AtAi[2] = -AtA[2]/det;

//    if(dbgcnt < 100)
//    {
//        qDebug() << "dirgrad2" << dbgcnt << dbgx << dbgy << det;
//        for(int d=0; d<2; d++)
//            if(isnanf(AtAi[d*2+0]) || isnanf(AtAi[d*2+1]))
//                throw GradientException("nan AtAi");
//    }

    // inv(A'*A)*A'
    float AtAiA[2*MAX_NEIGHBORS];
    for(int i=0; i<2; i++)
    {
        for(int d=0; d<vcount; d++)
        {
            AtAiA[i*vcount + d] = 0.f; // AtAi.row(i) * A.row(d)
            for(int k=0; k<2; k++)
                AtAiA[i*vcount + d] += AtAi[i*2 + k]*A[d*2 + k];
        }
    }

    // sol = inv(A'*A)*A'*b
    float sol[2];
    for(int i=0; i<2; i++)
    {
        sol[i] = 0.f; // AtAiA.row(i) * b
        for(int d=0; d<vcount; d++)
            sol[i] += AtAiA[i*vcount + d] * b[d];
    }

    // check result: |Ax-b| ~ 0 ?
    float errors[MAX_NEIGHBORS];
    for(int i=0; i<vcount; ++i)
        errors[i] = A[i*2+0]*sol[0] + A[i*2+1]*sol[1] - b[i];
    float sum=0.f, absum=0.f;
    for(int i=0; i<vcount; ++i)
    {
        sum += errors[i];
        absum += fabsf(errors[i]);
    }
//    if(dbgcnt < 100)
//        qDebug() << "dirgrad3" << sol[0] << sol[1] << sum << absum;

    return sol[0];  // sol = [gradient, constant]
}
