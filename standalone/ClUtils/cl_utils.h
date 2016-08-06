#ifndef CL_UTILS_H
#define CL_UTILS_H

#include "Share/project_common.h"
#include "cl_macros.h"
#include "clsetup.h"

#ifdef CL_VERSION_1_2
#define OPENCL_1_2
#endif

cl_program BuildClProgram(cl_device_id device, cl_context context, const char* prog_name, const char* include_path);
cl_kernel CreateClkernel(cl_program program, const char* kern_name);
cl_mem CreateClImageFloat4(cl_context context, const cl_uint width, const cl_uint height, const cl_int read_write);
cl_mem CreateClBuffer(cl_context context, const cl_uint size, const cl_int read_write);
void ExitOnError(bool error, const char* msg);

void PrintPlatformInfo(cl_platform_id platform);
void PrintDeviceInfo(cl_device_id device);

#endif // CL_UTILS_H
