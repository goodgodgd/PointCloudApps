#ifndef CLSETUP_H
#define CLSETUP_H

#include <stdlib.h>
#include "Share/project_common.h"
#include "cl_macros.h"
#include "cl_utils.h"

#define DEBUG_FL_SIZE   100

class ClSetup
{
    enum eINDEX
    {
        PLATFORM_INDEX = 0,
        DEVICE_INDEX = 0
    };

public:
    ClSetup();
    static cl_device_id GetDevice();
    static cl_context GetContext();
    static cl_command_queue GetQueue();
    static void Destroy();

private:
    static void SetupCl(int platform_index=PLATFORM_INDEX, int device_index=DEVICE_INDEX);
    static cl_platform_id GetPlatformID(int index=0);
    static cl_device_id GetDeviceID(int index=0);
    static cl_context CreateContext();
    static cl_command_queue CreateCommandQueue();

    static bool bInit;
    static cl_platform_id      platform;
    static cl_device_id        device;
    static cl_context          context;
    static cl_command_queue    queue;
};

#endif // CLSETUP_H
