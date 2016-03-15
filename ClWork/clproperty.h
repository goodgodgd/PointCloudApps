#ifndef CLPROPERTY_H
#define CLPROPERTY_H

#include "Share/project_common.h"
#define MAX_PLATFORMS       3

class ClProperty
{
public:
    ClProperty();
    void QueryClProperties(bool b_printinfo=true);
    void PrintPlatformInfo(cl_platform_id platform);
    void PrintDeviceInfo(cl_device_id device, bool b_printinfo);
    cl_uint clNumPlatforms, clNumDevices[MAX_PLATFORMS];
    cl_uint clMaxUnits, clMaxDim, clClockSpeed;
    cl_ulong clGlobalMemSize, clLocalMemSize;
    size_t clGroupSize;
};

#endif // CLPROPERTY_H
