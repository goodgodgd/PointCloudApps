#include "clworker.h"

CLWorker::CLWorker()
{
    m_clprop.QueryClProperties(false);
    init_GPU_OpenCL();
}

void CLWorker::init_GPU_OpenCL( )
{
    if(SetupClPlatform()==CL_SUCCESS)
        qDebug() << "Create context and queue";
    else
        return;

    if(BuildClProgram()==CL_SUCCESS)
        qDebug() << "Create program object";
    else
        return;

    if(SetupClkernels()==CL_SUCCESS)
        qDebug() << "Create kernels";
    else
        return;

    if(SetupClMems()==CL_SUCCESS)
        qDebug() << "Create memory objects";
    else
        return;

    gwsize[0] = IMAGE_WIDTH;
    gwsize[1] = IMAGE_HEIGHT;
//    lwsize[0] = 32;
//    lwsize[1] = m_clprop.clGroupSize/lwsize[0];
    lwsize[0] = 16;
    lwsize[1] = 16;
    qDebug() << "CL global work size:" << gwsize[0] << gwsize[1] << "workgroup size:" << lwsize[0] << lwsize[1];
}

cl_int CLWorker::SetupClPlatform()
{
    cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
    cl_int status = CL_SUCCESS+1;

    // Setup the OpenCL Platform,
    // Get the first available platform. Use it as the default platform
    if(m_clprop.clNumPlatforms>0)
    {
        status = clGetPlatformIDs(1, &platform, NULL);
        LOG_OCL_ERROR(status, "Error # clGetPlatformIDs");
    }

    //Get the first available device
    if(m_clprop.clNumDevices[0]>0)
    {
        status = clGetDeviceIDs (platform, deviceType, 1, &device, NULL);
        LOG_OCL_ERROR(status, "Error # clGetDeviceIDs");
    }

    //Create an execution context for the selected platform and device.
    cl_context_properties cps[3] =
    {
        CL_CONTEXT_PLATFORM,
        (cl_context_properties)platform,
        0
    };

    context = clCreateContext(cps, 1, &device, NULL, NULL, &status);
    LOG_OCL_ERROR(status, "Error # clCreateContextFromType" );

    // Create command queue
    commandQueue = clCreateCommandQueue(context, device, 0, &status);
    LOG_OCL_ERROR(status, "Error # clCreateCommandQueue" );

    return status;
}

cl_int CLWorker::BuildClProgram()
{
    // Get size of kernel source
    FILE* fp = fopen("../PCApps/Kernels/kernels.cl", "r");
    fseek(fp, 0, SEEK_END);
    size_t programSize = ftell(fp);
    rewind(fp);

    // Read kernel source into buffer
    char* programBuffer = (char*) malloc(programSize + 1);
    programBuffer[programSize] = '\0';
    fread(programBuffer, sizeof(char), programSize, fp);
    fclose(fp);

    // Create program from buffer
    cl_int status;
    program = clCreateProgramWithSource(context, 1,
           (const char**) &programBuffer, &programSize, &status);
    free(programBuffer);

    // Build the program
    status = clBuildProgram(program, 1, &device, NULL, NULL, NULL);
    if(status != CL_SUCCESS)
    {
        if(status == CL_BUILD_PROGRAM_FAILURE)
            LOG_OCL_COMPILER_ERROR(program, device);
        LOG_OCL_ERROR(status, "clBuildProgram Failed" );
    }
    return status;
}


cl_int CLWorker::SetupClkernels()
{
    cl_int status;
    // Create the OpenCL kernel
    gd_kernel = clCreateKernel(program, "mean_kernel", &status);
    LOG_OCL_ERROR(status, "clCreateKernel Failed" );

    return status;
}

cl_int CLWorker::SetupClMems()
{
    cl_int status;
    //Intermediate reusable cl buffers
    cl_image_format image_format;
    image_format.image_channel_data_type = CL_FLOAT;
    image_format.image_channel_order = CL_RGBA;

#ifdef OPENCL_1_2
    cl_image_desc image_desc;
    image_desc.image_type = CL_MEM_OBJECT_IMAGE2D;
    image_desc.image_width = IMAGE_WIDTH;
    image_desc.image_height = IMAGE_HEIGHT;
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
    ocl_raw = clCreateImage(
#else
    ocl_raw = clCreateImage2D(
#endif
        context,
        CL_MEM_READ_ONLY,
        &image_format,
#ifdef OPENCL_1_2
        &image_desc,
#else
        IMAGE_WIDTH, IMAGE_HEIGHT, 0,
#endif
        NULL,
        &status);
    LOG_OCL_ERROR(status, "clCreateImage Failed" );

#ifdef OPENCL_1_2
    //Note when the host_ptr is NULL row_pitch and slice_pith should be set to 0;
    //Otherwise you will get a CL_INVALID_IMAGE_DESCRIPTOR error
    image_desc.image_row_pitch = 0;
    image_desc.image_slice_pitch = 0;
    ocl_filtered_image = clCreateImage(
#else
    ocl_filtered_image = clCreateImage2D(
#endif
        context,
        CL_MEM_WRITE_ONLY,
        &image_format,
#ifdef OPENCL_1_2
        &image_desc,
#else
        IMAGE_WIDTH, IMAGE_HEIGHT, 0,
#endif
        NULL,
        &status);
    LOG_OCL_ERROR(status, "clCreateImage Failed" );

    //Create OpenCL device output buffer
    return status;
}

void CLWorker::ComputeNormal(cl_float4* pointCloud, cl_float4* normalCloud)
{
    // Copy host buffer to input image object
    size_t origin[3];
    size_t region[3];
    cl_int status = 0;
    origin[0] = origin[1] = origin[2] = 0;
    region[0] = IMAGE_WIDTH; region[1] = IMAGE_HEIGHT; region[2] = 1;
    status = clEnqueueWriteImage(
                        commandQueue,       // command queue
                        ocl_raw,            // device memory
                        CL_TRUE,            // block until finish
                        origin,             // origin of image
                        region,             // region of image
                        0, 0,               // row pitch, slice pitch
                        (void*)pointCloud,  // source host memory
                        0, NULL, NULL);     // wait, event
    LOG_OCL_ERROR(status, "clEnqueueWriteImage failed" );

    // Run kernel
    cl_event wlist[2];
    status = clSetKernelArg(gd_kernel, 0, sizeof(cl_mem), (void*)&ocl_raw);
    status = clSetKernelArg(gd_kernel, 1, sizeof(cl_mem), (void*)&ocl_filtered_image);
    status = clEnqueueNDRangeKernel(
                        commandQueue,   // command queue
                        gd_kernel,      // kernel
                        2,              // dimension
                        NULL,           // global offset
                        gwsize,         // global work size
                        lwsize,         // local work size
                        0,              // # of wait lists
                        NULL,           // wait list
                        &wlist[0]);     // event output
    LOG_OCL_ERROR(status, "clEnqueueNDRangeKernel Failed" );
    clWaitForEvents(1, &wlist[0]);

    // Copy back output of kernel to host buffer
    origin[0] = origin[1] = origin[2] = 0;
    region[0] = IMAGE_WIDTH; region[1] = IMAGE_HEIGHT; region[2] = 1;
    status = clEnqueueReadImage(
                        commandQueue,       // command queue
                        ocl_filtered_image, // source device memory
                        CL_TRUE,            // block until finish
                        origin,             // origin of image
                        region,             // region of image
                        0, 0,               // row pitch, slice pitch
                        (void*)normalCloud, // host memory
                        0, NULL, NULL);     // wait, event
    LOG_OCL_ERROR(status, "clEnqueueReadImage failed" );

}
