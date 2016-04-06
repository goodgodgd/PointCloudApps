#include "clsetup.h"

bool ClSetup::b_init = false;
cl_platform_id      ClSetup::platform = NULL;
cl_device_id        ClSetup::device = NULL;
cl_context          ClSetup::context = NULL;
cl_command_queue    ClSetup::queue = NULL;

ClSetup::ClSetup()
{
}

cl_device_id ClSetup::GetDevice()
{
    if(b_init==false)
    {
        SetupCl();
        atexit(Destroy);
    }
    return device;
}

cl_context ClSetup::GetContext()
{
    if(b_init==false)
    {
        SetupCl();
        atexit(Destroy);
    }
    return context;
}

cl_command_queue ClSetup::GetQueue()
{
    if(b_init==false)
    {
        SetupCl();
        atexit(Destroy);
    }
    return queue;
}

void ClSetup::SetupCl(int platform_index, int device_index)
{
    platform = GetPlatformID(platform_index);
    PrintPlatformInfo(platform);
    device = GetDeviceID(device_index);
    PrintDeviceInfo(device);
    context = CreateContext();
    queue = CreateCommandQueue();
    b_init = true;
}

cl_platform_id ClSetup::GetPlatformID(int index)
{
    cl_int status = CL_SUCCESS;
    cl_uint num_platforms = 0;

    status = clGetPlatformIDs(0, NULL, &num_platforms);
    LOG_OCL_ERROR(status, "GetPlatformID: num_platforms");
    if(num_platforms <= index)
    {
        printf("ClSetup: %d platforms found, platforms[%d] is NOT found\n", num_platforms, index);
        exit(-1);
    }

    cl_platform_id *platforms = (cl_platform_id *)malloc(sizeof(cl_platform_id)*num_platforms);
    status = clGetPlatformIDs(num_platforms, platforms, NULL);
    LOG_OCL_ERROR(status, "GetPlatformID: platform");
    return platforms[index];
}

cl_device_id ClSetup::GetDeviceID(int index)
{
    if(platform==NULL)
        return NULL;
    cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
    cl_int status = CL_SUCCESS;
    cl_uint num_devices = 0;

    status = clGetDeviceIDs(platform, deviceType, 0, NULL, &num_devices);
    LOG_OCL_ERROR(status, "clGetDeviceIDs: num_devices");
    if(num_devices <= index)
    {
        printf("ClSetup: %d devices found, devices[%d] is NOT found\n", num_devices, index);
        exit(-1);
    }

    cl_device_id *devices = (cl_device_id *)malloc(sizeof(cl_device_id)*num_devices);
    status = clGetDeviceIDs(platform, deviceType, num_devices, devices, NULL);
    LOG_OCL_ERROR(status, "clGetDeviceIDs: device");
    return devices[index];
}

cl_context ClSetup::CreateContext()
{
    cl_int status = CL_SUCCESS;
    cl_context_properties cps[3] =
    {
        CL_CONTEXT_PLATFORM,
        (cl_context_properties)platform,
        0
    };

    cl_context context = clCreateContext(cps, 1, &device, NULL, NULL, &status);
    LOG_OCL_ERROR(status, "CreateContext");
    return context;
}

cl_command_queue ClSetup::CreateCommandQueue()
{
    cl_int status = CL_SUCCESS;
    cl_command_queue queue = clCreateCommandQueue(context, device, 0, &status);
    LOG_OCL_ERROR(status, "CreateCommandQueue");
    return queue;
}

void ClSetup::Destroy()
{
    if(queue!=NULL)
        clReleaseCommandQueue(queue);
    if(context!=NULL)
        clReleaseContext(context);
    if(device!=NULL)
        clReleaseDevice(device);
}
