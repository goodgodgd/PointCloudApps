#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include <Eigen/Eigen>
#include <math.h>
#include "project_common.h"
#include "ClWork/cloperators.h"
#include "Share/forsearchneigbhor.h"
#include "Share/fordescriptor.h"
#include "linearsolver.h"

#define NUM_VAR     6
#define PT_DIM      3
#define L_DIM       (NUM_VAR+PT_DIM)
#define L_WIDTH     (L_DIM+1)

#define L_INDEX(y,x)    ((y)*L_WIDTH+(x))

void TestDescriptor();
void GeneratePoints(cl_float4* A, int num_pts, DescType shape);
void ComputeDescriptor(cl_float4* neighborCloud, cl_float4* normalCloud, DescType* descCloud);
void SetUpperLeft(cl_float4* neighborCloud, cl_float4 centerpt, int offset, int num_pts, float* L);
void SetUpperRight(cl_float4 normal, float* L);
void SetLowerLeft(cl_float4 normal, float* L);
void SetRightVector(cl_float4* neighborCloud, cl_float4 ctpoint, cl_float4 ctnormal, int offset, int num_pts, float* L);
void GetDescriptorByEigenDecomp(float Avec[NUM_VAR], float egval[PT_DIM], float egvec[PT_DIM*PT_DIM]);
void SwapEigen(float egval[PT_DIM], float egvec[PT_DIM*PT_DIM], int src, int dst);

#endif // DESCRIPTOR_H
