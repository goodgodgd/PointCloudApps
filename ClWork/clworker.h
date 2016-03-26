#ifndef CLWORKER_H
#define CLWORKER_H

#include <stdio.h>
#include <QElapsedTimer>
#include <cassert>
#include "Share/project_common.h"
#include "ClUtils/cl_macros.h"
#include "ClWork/clproperty.h"
#include "Share/forsearchneigbhor.h"
#include "Share/fordescriptor.h"

#ifdef CL_VERSION_1_2
#define OPENCL_1_2
#endif


class CLWorker
{
public:
    CLWorker();
    ~CLWorker();
    void SearchNeighborPoints(cl_float4* srcPointCloud, cl_float radius_meter, cl_float focalLength
                              , cl_int* outNeighborIndices, cl_int* outNumNeighbors);
    void ComputeNormalWithNeighborPts(cl_float4* dstNormalCloud);
    void ComputeDescriptorWithNeighborPts(DescType* dstDescriptorCloud);

private:
    //Initialize OpenCL stuffs
    void SetupOpenCL();		// init GPU FE mem, intit CL runtime, build program,
                                    // build kernels, init CL images/buffers
    cl_int SetupClPlatform();
    cl_int BuildClProgram();
    cl_int CreateClkernels();
    cl_int CreateClImages();
    cl_int CreateClMemsAndSetMemSize();

    ClProperty          clprop;

    //OpenCL Stuffs
    cl_platform_id      platform;
    cl_device_id        device;
    cl_context          context;
    cl_command_queue    commandQueue;
    cl_program          program;
    cl_kernel           kernelNormal;
    cl_kernel           kernelDescriptor;
    cl_kernel           kernelNeighborPts;
    size_t				gwsize[2];// OpenCL global work size
    size_t				lwsize[2];// OpenCL local work size
    size_t              imgOrigin[3];
    size_t              imgRegion[3];
    cl_int              maxNeighbors;

    cl_mem              memPoints;
    cl_mem              memNormals;
    cl_mem              memDescriptors;
    cl_mem              memNeighborIndices;
    cl_mem              memNumNeighbors;
    int                 szDescriptors;
    int                 szNeighborIdcs;
    int                 szNumNeighbors;
};

#endif // CLWORKER_H
