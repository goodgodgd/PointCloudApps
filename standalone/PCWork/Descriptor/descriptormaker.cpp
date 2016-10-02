#include "descriptormaker.h"

DescriptorMaker::DescriptorMaker()
{
}

DescriptorMaker::~DescriptorMaker()
{
    clReleaseMemObject(memDescriptors);
    clReleaseKernel(kernel);
    clReleaseProgram(program);
}

void DescriptorMaker::Setup()
{
    SetupBase();
    sprintf(kernel_name, "%s/ClKernels/compute_descriptor.cl", PCApps_PATH);
    program = BuildClProgram(device, context, kernel_name, include_path);
    kernel = CreateClkernel(program, "compute_descriptor");
    gradKernel = CreateClkernel(program, "compute_gradient");

    descriptorData.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    prinAxesData.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    szDescriptors = descriptorData.ByteSize();
    szDescAxes = prinAxesData.ByteSize();
    memDescriptors = CreateClBuffer(context, szDescriptors, CL_MEM_READ_WRITE);
    memDescAxes = CreateClBuffer(context, szDescAxes, CL_MEM_READ_WRITE);
    bInit = true;
}

void DescriptorMaker::ComputeDescriptors(cl_mem memPoints, cl_mem memNormals
                                         , cl_mem memNeighborIndices, cl_mem memNumNeighbors, const cl_int maxNeighbors)
{
    ComputeCurvatures(memPoints, memNormals, memNeighborIndices, memNumNeighbors, maxNeighbors);
    ComputeGradients(memPoints, memNeighborIndices, memNumNeighbors, maxNeighbors);
}

void DescriptorMaker::ComputeCurvatures(cl_mem memPoints, cl_mem memNormals, cl_mem memNeighborIndices, cl_mem memNumNeighbors, cl_int maxNeighbors)
{
    if(bInit==false)
        Setup();
    DescType* descriptors = descriptorData.GetArrayPtr();
    AxesType* prinAxes = prinAxesData.GetArrayPtr();
    cl_int status = 0;

    cl_event wlist[2];
    status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void*)&memNormals);
    status = clSetKernelArg(kernel, 2, sizeof(cl_mem), (void*)&memNeighborIndices);
    status = clSetKernelArg(kernel, 3, sizeof(cl_mem), (void*)&memNumNeighbors);
    status = clSetKernelArg(kernel, 4, sizeof(cl_int), (void*)&maxNeighbors);
    status = clSetKernelArg(kernel, 5, sizeof(cl_mem), (void*)&memDescriptors);
    status = clSetKernelArg(kernel, 6, sizeof(cl_mem), (void*)&memDescAxes);
    status = clSetKernelArg(kernel, 7, sizeof(cl_mem), (void*)&memDebug);

    status = clEnqueueNDRangeKernel(
                        queue,          // command queue
                        kernel,         // kernel
                        2,              // dimension
                        NULL,           // global offset
                        gwsize,         // global work size
                        lwsize,         // local work size
                        0,              // # of wait lists
                        NULL,           // wait list
                        &wlist[0]);     // event output
    LOG_OCL_ERROR(status, "clEnqueueNDRangeKernel(kernel)" );
    clWaitForEvents(1, &wlist[0]);

    // copy back output of kernel to host buffer
    status = clEnqueueReadBuffer(
                        queue,          // command queue
                        memDescriptors, // device memory
                        CL_TRUE,        // block until finish
                        0,              // offset
                        szDescriptors,  // size
                        descriptors,    // dst host memory
                        0, NULL, NULL); // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memDescriptors)");
    status = clEnqueueReadBuffer(
                        queue,          // command queue
                        memDescAxes,    // device memory
                        CL_TRUE,        // block until finish
                        0,              // offset
                        szDescAxes,     // size
                        prinAxes,       // dst host memory
                        0, NULL, NULL); // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memDescAxes)");
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memDebug,           // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szDebug,            // size
                        debugBuffer,        // dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(debugBuffer)");
}

void DescriptorMaker::ComputeGradients(cl_mem memPoints, cl_mem memNeighborIndices, cl_mem memNumNeighbors, const cl_int maxNeighbors)
{
    if(bInit==false)
        Setup();
    const cl_float descRadius = DescriptorRadius();
    DescType* descriptors = descriptorData.GetArrayPtr();
    AxesType* prinAxes = prinAxesData.GetArrayPtr();
    cl_int status = 0;

    cl_event wlist[2];
    status = clSetKernelArg(gradKernel, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(gradKernel, 1, sizeof(cl_mem), (void*)&memNeighborIndices);
    status = clSetKernelArg(gradKernel, 2, sizeof(cl_mem), (void*)&memNumNeighbors);
    status = clSetKernelArg(gradKernel, 3, sizeof(cl_int), (void*)&maxNeighbors);
    status = clSetKernelArg(gradKernel, 4, sizeof(cl_float), (void*)&descRadius);
    status = clSetKernelArg(gradKernel, 5, sizeof(cl_mem), (void*)&memDescriptors);
    status = clSetKernelArg(gradKernel, 6, sizeof(cl_mem), (void*)&memDescAxes);
    status = clSetKernelArg(gradKernel, 7, sizeof(cl_mem), (void*)&memDebug);

    status = clEnqueueNDRangeKernel(
                        queue,          // command queue
                        gradKernel,         // gradKernel
                        2,              // dimension
                        NULL,           // global offset
                        gwsize,         // global work size
                        lwsize,         // local work size
                        0,              // # of wait lists
                        NULL,           // wait list
                        &wlist[0]);     // event output
    LOG_OCL_ERROR(status, "clEnqueueNDRangeKernel(gradKernel)" );
    clWaitForEvents(1, &wlist[0]);

    // copy back output of gradKernel to host buffer
    status = clEnqueueReadBuffer(
                        queue,          // command queue
                        memDescriptors, // device memory
                        CL_TRUE,        // block until finish
                        0,              // offset
                        szDescriptors,  // size
                        descriptors,    // dst host memory
                        0, NULL, NULL); // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memDescriptors)");
    status = clEnqueueReadBuffer(
                        queue,          // command queue
                        memDescAxes,    // device memory
                        CL_TRUE,        // block until finish
                        0,              // offset
                        szDescAxes,     // size
                        prinAxes,       // dst host memory
                        0, NULL, NULL); // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memDescAxes)");
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memDebug,           // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szDebug,            // size
                        debugBuffer,        // dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(debugBuffer)");

    qDebug() << "computeGradients";
}
