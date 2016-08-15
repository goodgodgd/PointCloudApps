#ifndef DESCGRADIENTMAKERBYCPU_H
#define DESCGRADIENTMAKERBYCPU_H

#include <Eigen/Eigen>
#include <cassert>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/forsearchneigbhor.h"
#include "Share/arraydata.h"
#include "Share/shared_data.h"
#include "ClUtils/cloperators.h"

struct GradientException
{
    GradientException(QString msg) : msg_(msg) {}
    QString msg_;
};

class DescGradientMakerByCpu
{
public:
    DescGradientMakerByCpu();
    void CopmuteGradient(const cl_float4* pointCloud, const DescType* curvatures, AxesType* descAxes
                         , const cl_int* neighborIndices, const cl_int* numNeighbors
                         , const int maxNeighbs, const float descRadius);
    DescType* GetGradDesc() { return descriptorArray.GetArrayPtr(); }

private:
    void CopmuteEachGradient(const int ctidx, const cl_float4* pointCloud
                             , const DescType* curvatures, AxesType* descAxes
                             , const cl_int* neighborIndices, const cl_int* numNeighbors
                             , const int maxNeighbs, const float descRadius);
    float DirectedGradient(const cl_float4* pointCloud, const DescType* curvatures
                           , const cl_float4 thispoint, const cl_float8 thisaxes, const bool majorAxis
                           , const cl_int* neighborIndices, const int niOffset
                           , const int numNeighb, const float descRadius);

    ArrayData<cl_float4> descriptorArray;

};

#endif // DESCGRADIENTMAKERBYCPU_H
