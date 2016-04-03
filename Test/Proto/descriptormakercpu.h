#ifndef SHAPEDESCRIPTOR_H
#define SHAPEDESCRIPTOR_H

#include <Eigen/Eigen>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/forsearchneigbhor.h"
#include "ClUtils/cloperators.h"
#include "Test/linearsolver.h"

class DescriptorProto
{
#define EQUATION_SCALE      100.f
#define NUM_VAR                 6
#define PT_DIM                  3
#define L_DIM                   (NUM_VAR+PT_DIM)
#define L_WIDTH                 (L_DIM+1)
#define L_INDEX(y,x)            ((y)*L_WIDTH+(x))
#define DESC_EQUATION_SIZE      L_DIM*L_WIDTH

public:
    DescriptorProto();
    void ComputeDescriptorCloud(cl_float4* pointCloud, cl_float4* normalCloud
                                , cl_int* neighborIndices, cl_int* numNeighbors, int maxNeighbs
                                , DescType* descriptorCloud);
//protected:
    DescType ComputeEachDescriptor(cl_float4& ctpoint, cl_float4& ctnormal
                                , cl_float4* pointCloud, cl_int* neighborIndices, int niOffset, int numNeighbs, bool b_print=false);
private:
    bool IsInvalidPoint(cl_float4 point);
    void SetUpperLeft(cl_float4 ctpoint, cl_float4* pointCloud, cl_int* neighborIndices, int offset, int num_pts, float* L);
    void SetUpperRight(cl_float4 normal, float* L);
    void SetLowerLeft(cl_float4 normal, float* L);
    void SetRightVector(cl_float4 ctpoint, cl_float4 ctnormal
                                      , cl_float4* pointCloud, cl_int* neighborIndices, int offset, int num_pts, float* L);
    void SolveLinearEq(const int dim, float* Ab_io, float* x_out);
    DescType GetDescriptorByEigenDecomp(float Avec[NUM_VAR]);
    void SwapEigen(float egval[PT_DIM], float egvec[PT_DIM*PT_DIM], int src, int dst);
};

#endif // SHAPEDESCRIPTOR_H
