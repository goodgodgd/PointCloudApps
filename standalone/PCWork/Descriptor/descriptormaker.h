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
    DescType* GetDescriptor(){ return descriptorData.GetArrayPtr(); }
    AxesType* GetDescAxes(){ return descAxesData.GetArrayPtr(); }

    cl_mem memDescriptors;
    cl_mem memDescAxes;

private:
    void Setup();
    ArrayData<DescType> descriptorData;
    ArrayData<AxesType> descAxesData;
    cl_int szDescriptors;
    cl_int szDescAxes;
};

#endif // DESCRIPTORMAKER_H
