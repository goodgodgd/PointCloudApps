#ifndef DESCRIPTORMAKERBYCPU_H
#define DESCRIPTORMAKERBYCPU_H

#include <Eigen/Eigen>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/arraydata.h"
#include "Share/shared_data.h"
#include "ClUtils/cloperators.h"

struct DescriptorException
{
    DescriptorException(QString msg) : msg_(msg) {}
    QString msg_;
};

class DescriptorMakerByCpu
{
#define NUM_VAR                 6
#define PT_DIM                  3
#define L_DIM                   (NUM_VAR+PT_DIM)
#define L_WIDTH                 (L_DIM+1)
#define L_INDEX(y,x)            ((y)*L_WIDTH+(x))
#define DESC_EQUATION_SIZE      L_DIM*L_WIDTH

public:
    DescriptorMakerByCpu();
    void ComputeDescriptors(const cl_float4* pointCloud, const cl_float4* normalCloud
                            , const cl_int* neighborIndices, const cl_int* numNeighbors
                            , const int maxNeighbs);
    const DescType* GetDescriptors() { return descriptorArray.GetArrayPtr(); }
    const AxesType* GetDescAxes() { return axesArray.GetArrayPtr(); }
    static const float DescriptorRadius() { return DESC_RADIUS; }
    static const int DescriptorNeighbors() { return DESC_NEIGHBORS; }

private:
    void ComputeCurvature(const int pxidx, const cl_float4* pointCloud, const cl_float4* normalCloud
                            , const cl_int* neighborIndices, const cl_int* numNeighbors, const int maxNeighbs);
    bool IsInvalidPoint(cl_float4 point);
    void SetUpperLeft(const cl_float4& ctpoint, const cl_float4* pointCloud, const cl_int* neighborIndices
                      , const int offset, const int num_pts, Eigen::MatrixXf& LEA);
    void SetUpperRight(const cl_float4& normal, Eigen::MatrixXf& linEq);
    void SetLowerLeft(const cl_float4& normal, Eigen::MatrixXf& linEq);
    void SetRightVector(const cl_float4& ctpoint, const cl_float4* pointCloud, const cl_float4& ctnormal
                        , const cl_int* neighborIndices, const int offset, const int num_pts, Eigen::VectorXf& linEq);
    void SortEigens(Eigen::Vector3f& eval, Eigen::Matrix3f& evec);
    void SwapEigen(Eigen::Vector3f& egval, Eigen::Matrix3f& egvec, const int src, const int dst);

    void CopmuteGradient(const int ctidx, const cl_float4* pointCloud
                         , const cl_int* neighborIndices, const cl_int* numNeighbors
                         , const int maxNeighbs, const float descRadius);
    float DirectedGradient(const cl_float4* pointCloud, const DescType* descriptors
                           , const cl_float4 thispoint, const AxesType& descAxes, const bool majorAxis
                           , const cl_int* neighborIndices, const int niOffset
                           , const int numNeighb, const float descRadius);

    ArrayData<DescType> descriptorArray;
    ArrayData<AxesType> axesArray;
};

#endif // DESCRIPTORMAKERBYCPU_H
