#include "clproperty.h"

ClProperty::ClProperty()
{
}

void ClProperty::QueryClProperties(bool b_printinfo)
{
    cl_int           clError;

    // Get the Number of Platforms available
    // Note that the second parameter "platforms" is set to NULL. If this is NULL then this argument is ignored
    // and the API returns the total number of OpenCL platforms available.
    cl_platform_id * platforms = NULL;
    clNumPlatforms = 0;

    if ((clGetPlatformIDs(0, NULL, &clNumPlatforms)) == CL_SUCCESS)
    {
        if(b_printinfo)
            printf("Number of OpenCL platforms available in the system: %d\n", clNumPlatforms);
        platforms = (cl_platform_id *)malloc(sizeof(cl_platform_id)*clNumPlatforms);
        if(clGetPlatformIDs(clNumPlatforms, platforms, NULL) != CL_SUCCESS)
        {
            free(platforms);
            printf("Error in call to clGetPlatformIDs_2....\n Exiting");
            return;
        }
    }
    else
    {
        printf("Error in call to clGetPlatformIDs_1....\n Exiting");
        return;
    }

    if (clNumPlatforms == 0)
    {
        printf("No OpenCL Platforms Found ....\n Exiting");
        return;
    }

    clNumPlatforms = smin(clNumPlatforms, MAX_PLATFORMS);

    // We have obtained one platform here.
    // Lets enumerate the devices available in this Platform.
    for (cl_uint idx=0;idx<clNumPlatforms; idx++)
    {
        if(b_printinfo)
        {
            printf("==================Platform No %d======================\n", idx);
            PrintPlatformInfo(platforms[idx]);
            printf("======================================================\n\n");
        }

        cl_device_id    *devices;
        clNumDevices[idx] = 0;
        if(b_printinfo)
            printf("Printing OpenCL Device Info For Platform ID : %d\n", idx);
        clError = clGetDeviceIDs (platforms[idx], CL_DEVICE_TYPE_ALL, 0, NULL, &clNumDevices[idx]);
        if (clError != CL_SUCCESS)
        {
            printf("Error Getting number of devices... Exiting\n ");
            return;
        }
        // If successfull the clNumDevices[idx] contains the number of devices available in the platform
        // Now lets get all the device list. Before that we need to malloc devices
        devices = (cl_device_id *)malloc(sizeof(cl_device_id) * clNumDevices[idx]);
        clError = clGetDeviceIDs (platforms[idx], CL_DEVICE_TYPE_ALL, clNumDevices[idx], devices, &clNumDevices[idx]);
        if (clError != CL_SUCCESS)
        {
            free(devices);
            printf("Error Getting number of devices... Exiting\n ");
            return;
        }

        for (cl_uint dIndex = 0; dIndex < clNumDevices[idx]; dIndex++)
        {
            if(b_printinfo)
                printf("------------------Device No %d----------------------\n", dIndex);
            PrintDeviceInfo(devices[dIndex], b_printinfo);
            if(b_printinfo)
                printf("----------------------------------------------------\n\n");
        }
        free(devices);
    }

    free(platforms);
}

void ClProperty::PrintPlatformInfo(cl_platform_id platform)
{
    char queryBuffer[1024];
    cl_int clError;

    clError = clGetPlatformInfo (platform, CL_PLATFORM_NAME, 1024, &queryBuffer, NULL);
    if(clError == CL_SUCCESS)
    {
        printf("CL_PLATFORM_NAME   : %s\n", queryBuffer);
    }
    clError = clGetPlatformInfo (platform, CL_PLATFORM_VENDOR, 1024, &queryBuffer, NULL);
    if(clError == CL_SUCCESS)
    {
        printf("CL_PLATFORM_VENDOR : %s\n", queryBuffer);
    }
    clError = clGetPlatformInfo (platform, CL_PLATFORM_VERSION, 1024, &queryBuffer, NULL);
    if (clError == CL_SUCCESS)
    {
        printf("CL_PLATFORM_VERSION: %s\n", queryBuffer);
    }
    clError = clGetPlatformInfo (platform, CL_PLATFORM_PROFILE, 1024, &queryBuffer, NULL);
    if (clError == CL_SUCCESS)
    {
        printf("CL_PLATFORM_PROFILE: %s\n", queryBuffer);
    }
    clError = clGetPlatformInfo (platform, CL_PLATFORM_EXTENSIONS, 1024, &queryBuffer, NULL);
    if (clError == CL_SUCCESS)
    {
        printf("CL_PLATFORM_EXTENSIONS: %s\n", queryBuffer);
    }
    return;
}

void ClProperty::PrintDeviceInfo(cl_device_id device, bool b_printinfo)
{
    char queryBuffer[1024];
    cl_int clError;
    clError = clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(queryBuffer), &queryBuffer, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_NAME: %s\n", queryBuffer);
    queryBuffer[0] = '\0';
    clError = clGetDeviceInfo(device, CL_DEVICE_VENDOR, sizeof(queryBuffer), &queryBuffer, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_VENDOR: %s\n", queryBuffer);
    queryBuffer[0] = '\0';
    clError = clGetDeviceInfo(device, CL_DRIVER_VERSION, sizeof(queryBuffer), &queryBuffer, NULL);
    if(b_printinfo)
        printf("CL_DRIVER_VERSION: %s\n", queryBuffer);
    queryBuffer[0] = '\0';
    clError = clGetDeviceInfo(device, CL_DEVICE_VERSION, sizeof(queryBuffer), &queryBuffer, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_VERSION: %s\n", queryBuffer);
    queryBuffer[0] = '\0';
    clError = clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(int), &clMaxUnits, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_MAX_COMPUTE_UNITS: %d\n", clMaxUnits);
    clError = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(int), &clMaxDim, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS: %d\n", clMaxDim);

    // Maximum number of work-items that can be specified in each dimension of the work-group
    size_t* querySizes = new size_t[clMaxDim];
    clError = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(size_t)*clMaxDim, querySizes, NULL);
    if(b_printinfo)
    {
        if(clMaxDim==0)
            printf("CL_DEVICE_MAX_WORK_ITEM_SIZES: NULL\n");
        else if(clMaxDim==1)
            printf("CL_DEVICE_MAX_WORK_ITEM_SIZES: %d\n", (int)querySizes[0]);
        else if(clMaxDim==2)
            printf("CL_DEVICE_MAX_WORK_ITEM_SIZES: %d, %d\n", (int)querySizes[0], (int)querySizes[1]);
        else
            printf("CL_DEVICE_MAX_WORK_ITEM_SIZES: %d, %d, %d\n", (int)querySizes[0], (int)querySizes[1], (int)querySizes[2]);
    }

    // Maximum number of work-items in a work-group executing a kernel on a single compute unit, using the data parallel execution model
    clError = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &clGroupSize, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_MAX_WORK_GROUP_SIZE: %d\n", (int)clGroupSize);
    clError = clGetDeviceInfo(device, CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(cl_uint), &clClockSpeed, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_MAX_CLOCK_FREQUENCY: %d\n", (int)clClockSpeed);
    clError = clGetDeviceInfo(device, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(cl_ulong), &clGlobalMemSize, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_GLOBAL_MEM_SIZE: %d\n", (int)clGlobalMemSize);
    clError = clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(cl_ulong), &clLocalMemSize, NULL);
    if(b_printinfo)
        printf("CL_DEVICE_LOCAL_MEM_SIZE: %d\n", (int)clLocalMemSize);
}
