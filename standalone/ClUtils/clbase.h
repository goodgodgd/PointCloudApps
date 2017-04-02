#ifndef CLBASE_H
#define CLBASE_H

#include "clsetup.h"

class ClBase
{
public:
    ClBase()
        : bInit(false)
        , device(nullptr)
        , context(nullptr)
        , queue(nullptr)
        , program(nullptr)
        , kernel(nullptr)
        , memDebug(nullptr)
        , szDebug(0)
    {}

    virtual ~ClBase() {}

protected:
    void SetupBase()
    {
        gwsize[0] = IMAGE_WIDTH;
        gwsize[1] = IMAGE_HEIGHT;
#if (defined SCALE_VAR)
        if(SCALE_VAR==4)
        {
            lwsize[0] = 8;
            lwsize[1] = 8;
        }
        else
        {
            lwsize[0] = 16;
            lwsize[1] = 16;
        }
#else
        lwsize[0] = 16;
        lwsize[1] = 16;
#endif
        imgOrigin[0] = imgOrigin[1] = imgOrigin[2] = 0;
        imgRegion[0] = IMAGE_WIDTH;
        imgRegion[1] = IMAGE_HEIGHT;
        imgRegion[2] = 1;

        device = ClSetup::GetDevice();
        context = ClSetup::GetContext();
        queue = ClSetup::GetQueue();

        szDebug = DEBUG_FL_SIZE*sizeof(cl_float);
        memDebug = CreateClBuffer(context, szDebug, CL_MEM_WRITE_ONLY);

        sprintf(include_path, "-I%s/ClKernels", PCApps_PATH);
    }

    cl_float debugBuffer[DEBUG_FL_SIZE];

    void Setup();
    bool bInit;
    cl_device_id device;
    cl_context context;
    cl_command_queue queue;
    cl_program program;
    cl_kernel kernel;
    size_t gwsize[2];
    size_t lwsize[2];
    size_t imgOrigin[3];
    size_t imgRegion[3];
    cl_mem memDebug;
    cl_int szDebug;
    char include_path[100];
    char kernel_name[100];
};

#endif // CLBASE_H
