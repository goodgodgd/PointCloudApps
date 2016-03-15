#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include <Eigen/Eigen>
#include <math.h>
#include "Share/project_common.h"
#include "ClWork/cloperators.h"
#include "Share/forsearchneigbhor.h"
#include "Share/fordescriptor.h"
#include "linearsolver.h"


void TestDescriptor();
int CreatePointCloud(cl_float4 trueDesc, cl_float4* pointCloud);
void TransformPointCloud(cl_float4 rotation, cl_float4 translation, cl_float4* pointCloud, int num_pts, cl_float4& normalvt);
void ComputeDescriptor(cl_float4 ctpoint, cl_float4 ctnormal, cl_float4* neighborCloud, int num_pts, DescType& descriptor);
void SetUpperLeft(cl_float4* neighborCloud, cl_float4 centerpt, int offset, int num_pts, float* L);
void SetUpperRight(cl_float4 normal, float* L);
void SetLowerLeft(cl_float4 normal, float* L);
void SetRightVector(cl_float4* neighborCloud, cl_float4 ctpoint, cl_float4 ctnormal, int offset, int num_pts, float* L);
void GetDescriptorByEigenDecomp(float Avec[NUM_VAR], float egval[PT_DIM], float egvec[PT_DIM*PT_DIM]);
void SwapEigen(float egval[PT_DIM], float egvec[PT_DIM*PT_DIM], int src, int dst);

#endif // DESCRIPTOR_H
