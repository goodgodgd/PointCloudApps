#ifndef CLBASE_H
#define CLBASE_H

#include "clsetup.h"

class ClBase
{
public:
    ClBase();
    void SetupBase()
    {
        gwsize[0] = IMAGE_WIDTH;
        gwsize[1] = IMAGE_HEIGHT;
        lwsize[0] = 16;
        lwsize[1] = 16;
        imgOrigin[0] = imgOrigin[1] = imgOrigin[2] = 0;
        imgRegion[0] = IMAGE_WIDTH;
        imgRegion[1] = IMAGE_HEIGHT;
        imgRegion[2] = 1;

        device = ClSetup::GetDevice();
        context = ClSetup::GetContext();
        queue = ClSetup::GetQueue();

        program = BuildClProgram(device, context, "../PCApps/ClKernels/compute_normal_vector.cl", "-I../PCApps/ClKernels");
        kernel = CreateClkernel(program, "compute_normal_vector");

        memNormals = CreateClImageFloat4(context, IMAGE_WIDTH, IMAGE_HEIGHT, CL_MEM_READ_WRITE);

        szDebug = DEBUG_FL_SIZE*sizeof(cl_float);
        memDebug = CreateClBuffer(context, szDebug, CL_MEM_WRITE_ONLY);

        b_init = true;
    }

    cl_float debugBuffer[DEBUG_FL_SIZE];

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
    cl_int szDebug;
    cl_mem memDebug;

};

#endif // CLBASE_H
