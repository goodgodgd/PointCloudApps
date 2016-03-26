#ifndef DESCRIPTORMAKER_H
#define DESCRIPTORMAKER_H

#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "ClUtils/cl_macros.h"
#include "ClUtils/clsetup.h"
#include "ClUtils/cl_utils.h"

class DescriptorMaker
{
public:
    DescriptorMaker();
    ~DescriptorMaker();
    void ComputeDescriptor(cl_mem memPoints, cl_mem memNormals, cl_mem memNeighborIndices, cl_mem memNumNeighbors, cl_int maxNeighbors
                           , DescType* descriptorCloud_out);

private:
    void Setup();
    bool b_init;
    cl_device_id device;
    cl_context context;
    cl_command_queue queue;
    cl_program program;
    cl_kernel kernel;
    size_t gwsize[2];
    size_t lwsize[2];
    size_t imgOrigin[3];
    size_t imgRegion[3];

    cl_mem memDescriptors;
    cl_int szDescriptors;

};

#endif // DESCRIPTORMAKER_H
