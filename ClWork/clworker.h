#ifndef CLWORKER_H
#define CLWORKER_H

#include <stdio.h>
#include "project_common.h"
#include "ClWork/ocl_macros.h"
#include "ClWork/clproperty.h"


#ifdef CL_VERSION_1_2
#define OPENCL_1_2
#endif

class CLWorker
{
public:
    CLWorker();
    ~CLWorker();
    void ComputeNormal(cl_float4* pointCloud, cl_float radius_meter, cl_float focal_length, cl_float4* normalCloud);

private:
    //Initialize OpenCL stuffs
    void	SetupOpenCL();		// init GPU FE mem, intit CL runtime, build program,
                                    // build kernels, init CL images/buffers
    cl_int	SetupClPlatform();
    cl_int  BuildClProgram();
    cl_int  CreateClkernels();
    cl_int  CreateClMems();

    ClProperty                  clprop;

    //OpenCL Stuffs
    cl_platform_id              platform;
    cl_device_id                device;
    cl_context                  context;
    cl_command_queue            commandQueue;
    cl_program                  program;
    cl_kernel                   kernelNormal;
    size_t						gwsize[2];// OpenCL global work size
    size_t						lwsize[2];// OpenCL local work size
    cl_mem                      memPoints, memNormals;

};

#endif // CLWORKER_H
