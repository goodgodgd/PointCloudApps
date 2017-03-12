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
    void ComputeDescriptors(cl_mem memPoints, cl_mem memNormals
                            , cl_mem memNeighborIndices, cl_mem memNumNeighbors, const cl_int maxNeighbors);
    DescType* GetDescriptor(){ return descriptorData.GetArrayPtr(); }
    AxesType* GetDescAxes(){ return prinAxesData.GetArrayPtr(); }
    static const float DescriptorRadius() { return DESC_RADIUS; }
    static const int DescriptorNeighbors() { return NUM_NEIGHBORS; }

    cl_mem memDescriptors;
    cl_mem memDescAxes;

private:
    void ComputeCurvatures(cl_mem memPoints, cl_mem memNormals, cl_mem memNeighborIndices, cl_mem memNumNeighbors, const cl_int maxNeighbors);
    void ComputeGradients(cl_mem memPoints, cl_mem memNeighborIndices, cl_mem memNumNeighbors, const cl_int maxNeighbors);
    void Setup();

    cl_kernel gradKernel;
    ArrayData<DescType> descriptorData;
    ArrayData<AxesType> prinAxesData;
    cl_int szDescriptors;
    cl_int szDescAxes;
};

#endif // DESCRIPTORMAKER_H
