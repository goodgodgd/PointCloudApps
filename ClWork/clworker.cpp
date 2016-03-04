#include "clworker.h"

CLWorker::CLWorker()
{
    clprop.QueryClProperties(false);
    SetupOpenCL();
}

CLWorker::~CLWorker()
{
    clReleaseMemObject(memPoints);
    clReleaseMemObject(memNormals);
    clReleaseMemObject(memDescriptors);
    clReleaseMemObject(memNeighbors);
    clReleaseProgram(program);
    clReleaseCommandQueue(commandQueue);
    clReleaseContext(context);
}
void CLWorker::SetupOpenCL( )
{
    if(SetupClPlatform()==CL_SUCCESS)
        qDebug() << "Create context and queue";
    else
        return;

    if(BuildClProgram()==CL_SUCCESS)
        qDebug() << "Create program object";
    else
        return;

    if(CreateClkernels()==CL_SUCCESS)
        qDebug() << "Create kernels";
    else
        return;

    if(CreateClImages()==CL_SUCCESS)
        qDebug() << "Create memory objects";
    else
        return;

    if(CreateClMems()==CL_SUCCESS)
        qDebug() << "Create memory objects";
    else
        return;

    gwsize[0] = IMAGE_WIDTH;
    gwsize[1] = IMAGE_HEIGHT;
//    lwsize[0] = 32;
//    lwsize[1] = clprop.clGroupSize/lwsize[0];
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
    if(clprop.clNumPlatforms>0)
    {
        status = clGetPlatformIDs(1, &platform, NULL);
        LOG_OCL_ERROR(status, "Error # clGetPlatformIDs");
    }

    //Get the first available device
    if(clprop.clNumDevices[0]>0)
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
    FILE* fp = fopen("../PCApps/ClKernels/kernels.cl", "r");
    fseek(fp, 0, SEEK_END);
    size_t programSize = ftell(fp);
    rewind(fp);
    qDebug() << "Program size" << programSize;

    // Read kernel source into buffer
    char* programBuffer = (char*) malloc(programSize + 1);
    programBuffer[programSize] = '\0';
    fread(programBuffer, sizeof(char), programSize, fp);
    fclose(fp);

    // Create program from buffer
    cl_int status;
    program = clCreateProgramWithSource(context, 1,
           (const char**) &programBuffer, &programSize, &status);

    // Build the program
    status = clBuildProgram(program, 1, &device, "-I../PCApps/ClKernels", NULL, NULL);
    if(status != CL_SUCCESS)
    {
        if(status == CL_BUILD_PROGRAM_FAILURE)
            LOG_OCL_COMPILER_ERROR(program, device);
        LOG_OCL_ERROR(status, "clBuildProgram Failed" );
    }
    free(programBuffer);
    return status;
}


cl_int CLWorker::CreateClkernels()
{
    cl_int status;

    // create kernel to search neighbor points
    kernelNeighborPts = clCreateKernel(program, "search_neighbor_points", &status);
    LOG_OCL_ERROR(status, "clCreateKernel(search_neighbor_points) Failed" );

    // create kernel to compute normal vectors with point groups
    kernelNormal = clCreateKernel(program, "compute_normal_vector_with_neighbor_points", &status);
    LOG_OCL_ERROR(status, "clCreateKernel(compute_normal_vector_with_neighbor_points) Failed" );

    // create kernel to compute shape descriptor
    kernelDescriptor = clCreateKernel(program, "compute_descriptor_with_neighbor_points", &status);
    LOG_OCL_ERROR(status, "clCreateKernel(compute_descriptor_with_neighbor_points) Failed" );

    return status;
}

cl_int CLWorker::CreateClImages()
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
    memPoints = clCreateImage(
#else
    memPoints = clCreateImage2D(
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
    memNormals = clCreateImage(
#else
    memNormals = clCreateImage2D(
#endif
        context,
        CL_MEM_READ_WRITE,
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

cl_int CLWorker::CreateClMems()
{
    cl_int status;
    // Create memory buffers on the device
    memDescriptors = clCreateBuffer(context, CL_MEM_READ_WRITE,
                        IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(DescType), NULL, &status);
    LOG_OCL_ERROR(status, "clCreateBuffer(memDescriptors) Failed" );

    memNeighbors = clCreateBuffer(context, CL_MEM_READ_WRITE,
                        IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4)*NEIGHBORS_PER_POINT, NULL, &status);
    LOG_OCL_ERROR(status, "clCreateBuffer(memNeighbors) Failed" );

    return status;
}

void CLWorker::SearchNeighborPoints(cl_float4* srcPointCloud, cl_float radius_meter, cl_float focalLength, cl_int maxNumNeighbors
                                    , cl_float4* dstNeighbors)
{
    cl_int status = 0;
    cl_event wlist[2];
    QElapsedTimer eltimer;
    size_t origin[3];
    size_t region[3];
    origin[0] = origin[1] = origin[2] = 0;
    region[0] = IMAGE_WIDTH; region[1] = IMAGE_HEIGHT; region[2] = 1;

    // copy host buffer to input image object
    if(srcPointCloud!=NULL)
    {
        eltimer.start();
        status = clEnqueueWriteImage(
                            commandQueue,       // command queue
                            memPoints,          // device memory
                            CL_TRUE,            // block until finish
                            origin,             // origin of image
                            region,             // region of image
                            0, 0,               // row pitch, slice pitch
                            (void*)srcPointCloud,  // source host memory
                            0, NULL, NULL);     // wait, event
        LOG_OCL_ERROR(status, "clEnqueueWriteImage(memPoints) failed" );
        qDebug() << "   clEnqueueWriteImage took" << eltimer.nsecsElapsed()/1000 << "us";
    }

    // excute kernel
    eltimer.start();
    status = clSetKernelArg(kernelNeighborPts, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(kernelNeighborPts, 1, sizeof(cl_float), (void*)&radius_meter);
    status = clSetKernelArg(kernelNeighborPts, 2, sizeof(cl_float), (void*)&focalLength);
    status = clSetKernelArg(kernelNeighborPts, 3, sizeof(cl_int), (void*)&maxNumNeighbors);
    status = clSetKernelArg(kernelNeighborPts, 4, sizeof(cl_mem), (void*)&memNeighbors);

    status = clEnqueueNDRangeKernel(
                        commandQueue,   // command queue
                        kernelNeighborPts, // kernel
                        2,              // dimension
                        NULL,           // global offset
                        gwsize,         // global work size
                        lwsize,         // local work size
                        0,              // # of wait lists
                        NULL,           // wait list
                        &wlist[0]);     // event output
    LOG_OCL_ERROR(status, "clEnqueueNDRangeKernel(kernelNeighborPts) Failed");
    clWaitForEvents(1, &wlist[0]);
    qDebug() << "   clEnqueueNDRangeKernel took" << eltimer.nsecsElapsed()/1000 << "us";

    // copy back output of kernel to host buffer
    eltimer.start();
    status = clEnqueueReadBuffer(
                        commandQueue,   // command queue
                        memNeighbors,   // device memory
                        CL_TRUE,        // block until finish
                        0,              // offset
                        IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4)*NEIGHBORS_PER_POINT,  // size
                        dstNeighbors,   // dst host memory
                        0, NULL, NULL); // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memNeighbors) Failed");
    qDebug() << "   clEnqueueReadBuffer took" << eltimer.nsecsElapsed()/1000 << "us";
}

void CLWorker::ComputeNormalWithNeighborPts(cl_float4* srcNeighborPoints, cl_float4* dstNormalCloud)
{
    // Copy host buffer to input image object
    size_t origin[3];
    size_t region[3];
    cl_int status = 0;
    origin[0] = origin[1] = origin[2] = 0;
    region[0] = IMAGE_WIDTH; region[1] = IMAGE_HEIGHT; region[2] = 1;
    cl_int maxNumNeighbors = NEIGHBORS_PER_POINT;

    QElapsedTimer eltimer;
    if(srcNeighborPoints!=NULL)
    {
        eltimer.start();
        status = clEnqueueWriteBuffer(
                            commandQueue,   // command queue
                            memNeighbors,   // device memory
                            CL_TRUE,        // block until finish
                            0,              // offset
                            IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4)*NEIGHBORS_PER_POINT,  // size
                            srcNeighborPoints, // src host memory
                            0, NULL, NULL); // events
        LOG_OCL_ERROR(status, "clEnqueueWriteBuffer(memNeighbors) failed" );
        qDebug() << "   clEnqueueWriteBuffer took" << eltimer.nsecsElapsed()/1000 << "us";
    }

    eltimer.start();
    // excute kernel
    cl_event wlist[2];
    status = clSetKernelArg(kernelNormal, 0, sizeof(cl_mem), (void*)&memNeighbors);
    status = clSetKernelArg(kernelNormal, 1, sizeof(cl_int), (void*)&maxNumNeighbors);
    status = clSetKernelArg(kernelNormal, 2, sizeof(cl_mem), (void*)&memNormals);
    status = clEnqueueNDRangeKernel(
                        commandQueue,   // command queue
                        kernelNormal,   // kernel
                        2,              // dimension
                        NULL,           // global offset
                        gwsize,         // global work size
                        lwsize,         // local work size
                        0,              // # of wait lists
                        NULL,           // wait list
                        &wlist[0]);     // event output
    LOG_OCL_ERROR(status, "clEnqueueNDRangeKernel(kernelNormal) Failed" );
    clWaitForEvents(1, &wlist[0]);
    qDebug() << "   clEnqueueNDRangeKernel took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    // copy back output of kernel to host buffer
    origin[0] = origin[1] = origin[2] = 0;
    region[0] = IMAGE_WIDTH; region[1] = IMAGE_HEIGHT; region[2] = 1;
    status = clEnqueueReadImage(
                        commandQueue,       // command queue
                        memNormals,         // source device memory
                        CL_TRUE,            // block until finish
                        origin,             // origin of image
                        region,             // region of image
                        0, 0,               // row pitch, slice pitch
                        (void*)dstNormalCloud, // host memory
                        0, NULL, NULL);     // wait, event
    LOG_OCL_ERROR(status, "clEnqueueReadImage(memNormals) failed" );
    qDebug() << "   clEnqueueReadImage took" << eltimer.nsecsElapsed()/1000 << "us";
}

void CLWorker::ComputeNormalWithNeighborPts(cl_float4* dstNormalCloud)
{
    ComputeNormalWithNeighborPts(NULL, dstNormalCloud);
}

void CLWorker::ComputeDescriptorWithNeighborPts(cl_float4* srcNeighborPoints, cl_float4* srcNormalCloud
                                                , DescType* dstDescriptorCloud)
{
    // Copy host buffer to input image object
    size_t origin[3];
    size_t region[3];
    cl_int status = 0;
    origin[0] = origin[1] = origin[2] = 0;
    region[0] = IMAGE_WIDTH; region[1] = IMAGE_HEIGHT; region[2] = 1;
    cl_int maxNumNeighbors = NEIGHBORS_PER_POINT;

    QElapsedTimer eltimer;
    if(srcNeighborPoints!=NULL)
    {
        eltimer.start();
        status = clEnqueueWriteBuffer(
                            commandQueue,   // command queue
                            memNeighbors,   // device memory
                            CL_TRUE,        // block until finish
                            0,              // offset
                            IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4)*NEIGHBORS_PER_POINT,  // size
                            srcNeighborPoints, // src host memory
                            0, NULL, NULL); // events
        LOG_OCL_ERROR(status, "clEnqueueWriteBuffer(memNeighbors) failed" );
        qDebug() << "   clEnqueueWriteBuffer took" << eltimer.nsecsElapsed()/1000 << "us";
    }

    if(srcNormalCloud!=NULL)
    {
        eltimer.start();
        status = clEnqueueWriteImage(
                            commandQueue,       // command queue
                            memNormals,         // device memory
                            CL_TRUE,            // block until finish
                            origin,             // origin of image
                            region,             // region of image
                            0, 0,               // row pitch, slice pitch
                            (void*)srcNormalCloud,  // source host memory
                            0, NULL, NULL);     // wait, event
        LOG_OCL_ERROR(status, "clEnqueueWriteImage(memNormals) failed" );
        qDebug() << "   clEnqueueWriteImage took" << eltimer.nsecsElapsed()/1000 << "us";
    }

    // excute kernel
    eltimer.start();
    cl_event wlist[2];
    status = clSetKernelArg(kernelDescriptor, 0, sizeof(cl_mem), (void*)&memNeighbors);
    status = clSetKernelArg(kernelDescriptor, 1, sizeof(cl_int), (void*)&maxNumNeighbors);
    status = clSetKernelArg(kernelDescriptor, 2, sizeof(cl_mem), (void*)&memNormals);
    status = clSetKernelArg(kernelDescriptor, 3, sizeof(cl_mem), (void*)&memDescriptors);
    status = clEnqueueNDRangeKernel(
                        commandQueue,   // command queue
                        kernelDescriptor,   // kernel
                        2,              // dimension
                        NULL,           // global offset
                        gwsize,         // global work size
                        lwsize,         // local work size
                        0,              // # of wait lists
                        NULL,           // wait list
                        &wlist[0]);     // event output
    LOG_OCL_ERROR(status, "clEnqueueNDRangeKernel(kernelDescriptor) Failed" );
    clWaitForEvents(1, &wlist[0]);
    qDebug() << "   clEnqueueNDRangeKernel took" << eltimer.nsecsElapsed()/1000 << "us";

    // copy back output of kernel to host buffer
    eltimer.start();
    status = clEnqueueReadBuffer(
                        commandQueue,   // command queue
                        memDescriptors, // device memory
                        CL_TRUE,        // block until finish
                        0,              // offset
                        IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(DescType),  // size
                        dstDescriptorCloud,   // dst host memory
                        0, NULL, NULL); // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memDescriptors) Failed");
    qDebug() << "   clEnqueueReadBuffer took" << eltimer.nsecsElapsed()/1000 << "us";
}

void CLWorker::ComputeDescriptorWithNeighborPts(DescType* dstDescriptorCloud)
{
    ComputeDescriptorWithNeighborPts(NULL, NULL, dstDescriptorCloud);
}










