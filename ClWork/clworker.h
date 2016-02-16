#ifndef CLWORKER_H
#define CLWORKER_H

#include <stdio.h>
#include "project_common.h"
#include "ClUtils/ocl_macros.h"
#include "ClUtils/clproperty.h"


//#ifdef CL_VERSION_1_2
//#define OPENCL_1_2
//#endif

class CLWorker
{
public:
    CLWorker();
    void ComputeNormal(cl_float4* pointCloud, cl_float4* normalCloud);

private:
    //Initialize OpenCL stuffs
    void	init_GPU_OpenCL();		// init GPU FE mem, intit CL runtime, build program,
                                    // build kernels, init CL images/buffers
    cl_int	SetupClPlatform();
    cl_int  BuildClProgram();
    cl_int  SetupClkernels();
    cl_int  SetupClMems();

    ClProperty                  m_clprop;

    //OpenCL Stuffs
    cl_platform_id              platform;
    cl_device_id                device;
    cl_context                  context;
    cl_command_queue            commandQueue;
    cl_program                  program;
    cl_kernel                   gd_kernel;
    size_t						gwsize[2];// OpenCL global work size
    size_t						lwsize[2];// OpenCL local work size
    cl_mem                      ocl_raw, ocl_filtered_image;

};

#endif // CLWORKER_H
