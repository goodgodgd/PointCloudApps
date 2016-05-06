#ifndef DESCRIPTORMAKER_H
#define DESCRIPTORMAKER_H

#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/arraydata.h"
#include "ClUtils/cl_macros.h"
#include "ClUtils/clsetup.h"
#include "ClUtils/cl_utils.h"
#include "ClUtils/clbase.h"

class DescriptorMaker : public ClBase
{
public:
    DescriptorMaker();
    ~DescriptorMaker();
    void ComputeDescriptor(cl_mem memPoints, cl_mem memNormals, cl_mem memNeighborIndices, cl_mem memNumNeighbors, cl_int maxNeighbors);
    cl_float4* GetDescriptor();

private:
    void Setup();
    cl_mem memDescriptors;
    cl_int szDescriptors;
    ArrayData<cl_float4> descriptorData;
};

#endif // DESCRIPTORMAKER_H
