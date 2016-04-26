#ifndef DESCRIPTORTESTER_H
#define DESCRIPTORTESTER_H

#include "Proto/descriptormakercpu.h"

namespace Test
{

class TestDescriptor
{
#define NB_HALF_DIM     10
#define NB_DIM          (NB_HALF_DIM*2+1)
#define NUM_PTS         (NB_DIM*NB_DIM)
#define NB_IDX(r,c)     (r*NB_DIM+c)

    static int CreatePointsForDescriptor(cl_float4 trueDesc, cl_float4* pointCloud)
    {
        const float itv = 0.02f;
        cl_float4 point;

        for(int r=0; r<NB_DIM; r++)
        {
            for(int c=0; c<NB_DIM; c++)
            {
                point.x = (r-NB_HALF_DIM)*itv;
                point.y = (c-NB_HALF_DIM)*itv;
                point.z = trueDesc.x*point.x*point.x + trueDesc.y*point.y*point.y;
                pointCloud[NB_IDX(r,c)] = point;
            }
        }
        return NUM_PTS;
    }

    static void TransformPointsForDescriptor(cl_float4 rotation, cl_float4 translation, cl_float4* pointCloud, int num_pts, cl_float4& normalvt)
    {
        cl_float4 rotmat[3];
        rotmat[0] = cl_float4{1,0,0,0};
        rotmat[1] = cl_float4{0,cosf(rotation.x),-sinf(rotation.x),0};
        rotmat[2] = cl_float4{0,sinf(rotation.x),cosf(rotation.x),0};
        cl_float4 npoint;

        for(int i=0; i<num_pts; i++)
        {
            npoint = pointCloud[i];
            pointCloud[i].x = clDot(rotmat[0], npoint);
            pointCloud[i].y = clDot(rotmat[1], npoint);
            pointCloud[i].z = clDot(rotmat[2], npoint);
            pointCloud[i] = pointCloud[i] + translation;
        }

        npoint = normalvt;
        normalvt.x = clDot(rotmat[0], npoint);
        normalvt.y = clDot(rotmat[1], npoint);
        normalvt.z = clDot(rotmat[2], npoint);
    }

public:
    static void ComputeEachDescriptor()
    {
        cl_float4 trueDesc = (cl_float4){0.3f, 0.2f, 0, 0};
        cl_float4 neighborCloud[NUM_PTS];
        cl_int neibIndices[NUM_PTS];
        cl_float4 translation = (cl_float4){1, 2, 3, 0};
        cl_float4 rotation = (cl_float4){0.5f, 0, 0, 0};
        cl_float4 normalvt = (cl_float4){0.f, 0.f, 1.f, 0};
        cl_float4 descriptor;
        for(int i=0; i<NUM_PTS; i++)
            neibIndices[i] = i;
        DescriptorMakerCpu descMaker;

        CreatePointsForDescriptor(trueDesc, neighborCloud);
        TransformPointsForDescriptor(rotation, translation, neighborCloud, NUM_PTS, normalvt);
        descriptor = descMaker.ComputeEachDescriptor(translation, normalvt, neighborCloud, neibIndices, 0, NUM_PTS);

        qDebug() << "TestDescriptor";
        qDebug() << "   true descriptor:" << trueDesc;
        qDebug() << "   computed descriptor" << descriptor;
    }

};


}

#endif // DESCRIPTORTESTER_H
