#include "cl_utils.h"

cl_program BuildClProgram(cl_device_id device, cl_context context, const char* prog_name, const char* include_path)
{
    cl_program program;
    cl_int status;

    // get size of kernel source
    FILE* fp = fopen(prog_name, "r");
    ExitOnError(fp==NULL, "Program file NOT found");
    fseek(fp, 0, SEEK_END);
    size_t prog_size = ftell(fp);
    rewind(fp);
    qDebug() << "Program name:" << prog_name << "/ size" << prog_size;

    // read kernel source into buffer
    char* prog_buffer = (char*) malloc(prog_size + 1);
    prog_buffer[prog_size] = '\0';
    fread(prog_buffer, sizeof(char), prog_size, fp);
    fclose(fp);

    program = clCreateProgramWithSource(context, 1,
           (const char**) &prog_buffer, &prog_size, &status);
    LOG_OCL_ERROR(status, "clCreateProgramWithSource");


    status = clBuildProgram(program, 1, &device, include_path, NULL, NULL);
    if(status != CL_SUCCESS)
    {
        if(status == CL_BUILD_PROGRAM_FAILURE)
            LOG_OCL_COMPILER_ERROR(program, device);
        LOG_OCL_ERROR(status, "clBuildProgram");
    }

    free(prog_buffer);
    return program;
}

cl_kernel CreateClkernel(cl_program program, const char* kern_name)
{
    cl_int status;
    cl_kernel kernel = clCreateKernel(program, kern_name, &status);
    LOG_OCL_ERROR(status, "clCreateKernel");
    return kernel;
}

cl_mem CreateClImageFloat4(cl_context context, const cl_uint width, const cl_uint height, const cl_int read_write)
{
    cl_int status;
    cl_mem mem_image;
    //Intermediate reusable cl buffers
    cl_image_format image_format;
    image_format.image_channel_data_type = CL_FLOAT;
    image_format.image_channel_order = CL_RGBA;

#ifdef OPENCL_1_2
    cl_image_desc image_desc;
    image_desc.image_type = CL_MEM_OBJECT_IMAGE2D;
    image_desc.image_width = width;
    image_desc.image_height = height;
    image_desc.image_depth = 1;
    image_desc.image_array_size = 1;
    //Note when the host_ptr is NULL row_pitch and slice_pith should be set to 0;
    //Otherwise you will get a CL_INVALID_IMAGE_DESCRIPTOR error
    image_desc.image_row_pitch = 0;
    image_desc.image_slice_pitch = 0;
    image_desc.num_mip_levels = 0;
    image_desc.num_samples = 0;
    image_desc.buffer= NULL;
#endif


#ifdef OPENCL_1_2
    mem_image = clCreateImage(
#else
    mem_image = clCreateImage2D(
#endif
        context,
        read_write,
        &image_format,
#ifdef OPENCL_1_2
        &image_desc,
#else
        width, height, 0,
#endif
        NULL,
        &status);
    LOG_OCL_ERROR(status, "clCreateImage Failed");

    return mem_image;
}

cl_mem CreateClBuffer(cl_context context, const cl_uint size, const cl_int read_write)
{
    cl_int status;
    cl_mem memory = clCreateBuffer(context, read_write, size, NULL, &status);
    LOG_OCL_ERROR(status, "clCreateBuffer");
    return memory;
}

void ExitOnError(bool error, const char* msg)
{
    if(error==true)
    {
        printf("Error: %s\n", msg);
        exit(-1);
    }
}


void PrintPlatformInfo(cl_platform_id platform)
{
    char buffer[1024];
    cl_int status;

    buffer[0] = '\0';
    status = clGetPlatformInfo (platform, CL_PLATFORM_NAME, 1024, &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintPlatformInfo: CL_PLATFORM_NAME");
    printf("CL_PLATFORM_NAME   : %s\n", buffer);

    buffer[0] = '\0';
    status = clGetPlatformInfo (platform, CL_PLATFORM_VERSION, 1024, &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintPlatformInfo: CL_PLATFORM_VERSION");
    printf("CL_PLATFORM_VERSION: %s\n", buffer);
/*
    buffer[0] = '\0';
    status = clGetPlatformInfo (platform, CL_PLATFORM_VENDOR, 1024, &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintPlatformInfo: CL_PLATFORM_VENDOR");
    printf("CL_PLATFORM_VENDOR : %s\n", buffer);

    buffer[0] = '\0';
    status = clGetPlatformInfo (platform, CL_PLATFORM_PROFILE, 1024, &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintPlatformInfo: CL_PLATFORM_PROFILE");
    printf("CL_PLATFORM_PROFILE: %s\n", buffer);

    buffer[0] = '\0';
    status = clGetPlatformInfo (platform, CL_PLATFORM_EXTENSIONS, 1024, &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintPlatformInfo: CL_PLATFORM_EXTENSIONS");
    printf("CL_PLATFORM_EXTENSIONS: %s\n", buffer);
*/
}

void PrintDeviceInfo(cl_device_id device)
{
    char buffer[1024];
    cl_int status;
    cl_uint max_units, max_dims;
    cl_uint group_size, clock_speed, global_mem_size, local_mem_size;

    buffer[0] = '\0';
    status = clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(buffer), &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_NAME");
    printf("CL_DEVICE_NAME: %s\n", buffer);

    buffer[0] = '\0';
    status = clGetDeviceInfo(device, CL_DEVICE_VENDOR, sizeof(buffer), &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_VENDOR");
    printf("CL_DEVICE_VENDOR: %s\n", buffer);
/*
    buffer[0] = '\0';
    status = clGetDeviceInfo(device, CL_DRIVER_VERSION, sizeof(buffer), &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DRIVER_VERSION");
    printf("CL_DRIVER_VERSION: %s\n", buffer);
*/
    buffer[0] = '\0';
    status = clGetDeviceInfo(device, CL_DEVICE_VERSION, sizeof(buffer), &buffer, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_VERSION");
    printf("CL_DEVICE_VERSION: %s\n", buffer);

    status = clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(int), &max_units, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_MAX_COMPUTE_UNITS");
    printf("CL_DEVICE_MAX_COMPUTE_UNITS: %d\n", (int)max_units);
/*
    status = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(int), &max_dims, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS");
    printf("CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS: %d\n", (int)max_dims);
*/
    // Maximum number of work-items that can be specified in each dimension of the work-group
    size_t* each_dim = new size_t[max_dims];
    status = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(size_t)*max_dims, each_dim, NULL);
    printf("CL_DEVICE_MAX_WORK_ITEM_SIZES:");
    for(int i=0; i<max_dims; i++)
        printf(" %d", (int)each_dim[0]);
    printf("\n");
    delete[] each_dim;

    // Maximum number of work-items in a work-group executing a kernel on a single compute unit, using the data parallel execution model
    status = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &group_size, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_MAX_WORK_GROUP_SIZE");
    printf("CL_DEVICE_MAX_WORK_GROUP_SIZE: %d\n", (int)group_size);

    status = clGetDeviceInfo(device, CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(cl_uint), &clock_speed, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_MAX_CLOCK_FREQUENCY");
    printf("CL_DEVICE_MAX_CLOCK_FREQUENCY: %d\n", (int)clock_speed);
/*
    status = clGetDeviceInfo(device, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(cl_ulong), &global_mem_size, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_GLOBAL_MEM_SIZE");
    printf("CL_DEVICE_GLOBAL_MEM_SIZE: %d\n", (int)global_mem_size);

    status = clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(cl_ulong), &local_mem_size, NULL);
    LOG_OCL_ERROR(status, "PrintDeviceInfo: CL_DEVICE_LOCAL_MEM_SIZE");
    printf("CL_DEVICE_LOCAL_MEM_SIZE: %d\n", (int)local_mem_size);
*/
}
