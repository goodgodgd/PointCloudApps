#ifndef SHAPEDESCRIPTOR_H
#define SHAPEDESCRIPTOR_H

#include <Eigen/Eigen>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/forsearchneigbhor.h"
#include "ClWork/cloperators.h"
#include "Test/linearsolver.h"

#define EQUATION_SCALE      100.f

class ShapeDescriptor
{
public:
    ShapeDescriptor();
    void ComputeDescriptorCloud(cl_float4* pointCloud, cl_float4* normalCloud
                                , cl_int* neighborIndices, cl_int* numNeighbors, int maxNeighbs
                                , DescType* descriptorCloud);
    void SetDescriptorEquation(cl_float4* pointCloud, cl_float4* normalCloud
                                , cl_int* neighborIndices, cl_int* numNeighbors, int maxNeighbs
                                , cl_float* descriptorEquation);

private:
    bool IsInvalidPoint(cl_float4 point);
    void SetEachEquation(cl_float4& ctpoint, cl_float4& ctnormal
                                , cl_float4* pointCloud, cl_int* neighborIndices, int niOffset, int numNeighbs
                                , cl_float* equation);
    DescType ComputeEachDescriptor(cl_float4& ctpoint, cl_float4& ctnormal
                                , cl_float4* pointCloud, cl_int* neighborIndices, int niOffset, int numNeighbs, bool b_print=false);
    void SetUpperLeft(cl_float4 ctpoint, cl_float4* pointCloud, cl_int* neighborIndices, int offset, int num_pts, float* L);
    void SetUpperRight(cl_float4 normal, float* L);
    void SetLowerLeft(cl_float4 normal, float* L);
    void SetRightVector(cl_float4 ctpoint, cl_float4 ctnormal
                                      , cl_float4* pointCloud, cl_int* neighborIndices, int offset, int num_pts, float* L);
    void SolveLinearEq(const int dim, float* Ab_io, float* x_out);
    DescType GetDescriptorByEigenDecomp(float Avec[NUM_VAR]);
    void SwapEigen(float egval[PT_DIM], float egvec[PT_DIM*PT_DIM], int src, int dst);

    int dbg_x;
    int dbg_y;

};

#endif // SHAPEDESCRIPTOR_H
