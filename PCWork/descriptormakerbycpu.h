#ifndef DESCRIPTORMAKERBYCPU_H
#define DESCRIPTORMAKERBYCPU_H

#include <Eigen/Eigen>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/forsearchneigbhor.h"
#include "Share/shared_enums.h"
#include "Share/arraydata.h"
#include "ClUtils/cloperators.h"
#include "Test/Proto/descriptormakercpu.h"

class DescriptorMakerByCpu
{
#define EQUATION_SCALE          100.f
#define NUM_VAR                 6
#define PT_DIM                  3
#define L_DIM                   (NUM_VAR+PT_DIM)
#define L_WIDTH                 (L_DIM+1)
#define L_INDEX(y,x)            ((y)*L_WIDTH+(x))
#define DESC_EQUATION_SIZE      L_DIM*L_WIDTH

public:
    DescriptorMakerByCpu();
    void ComputeDescriptors(const cl_float4* pointCloud_, const cl_float4* normalCloud
                            , const cl_int* neighborIndices, const cl_int* numNeighbors, const int maxNeighbs);
    const DescType* GetDescriptors();

private:
    DescType ComputeEachDescriptor(const cl_float4& ctpoint, const cl_float4& ctnormal
                                   , const cl_int* neighborIndices, const int niOffset, const int numNeighbs);
    bool IsInvalidPoint(cl_float4 point);
    void SetUpperLeft(const cl_float4& ctpoint, const cl_int* neighborIndices, const int offset, const int num_pts
                      , Eigen::MatrixXf& linEq);
    void SetUpperRight(const cl_float4& normal, Eigen::MatrixXf& linEq);
    void SetLowerLeft(const cl_float4& normal, Eigen::MatrixXf& linEq);
    void SetRightVector(const cl_float4& ctpoint, const cl_float4& ctnormal
                        , const cl_int* neighborIndices, const int offset, const int num_pts, Eigen::VectorXf& linEq);
    DescType GetDescriptorByEigenDecomp(const Eigen::VectorXf& Avec);
    void SwapEigen(Eigen::Vector3f& egval, Eigen::Matrix3f& egvec, const int src, const int dst);

    ArrayData<DescType> descriptorArray;
    DescType* descriptors;
    const cl_float4* pointCloud;
};

#endif // DESCRIPTORMAKERBYCPU_H
