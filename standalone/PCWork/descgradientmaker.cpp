#include "descgradientmaker.h"

DescGradientMaker::DescGradientMaker()
{
}

DescGradientMaker::~DescGradientMaker()
{
    clReleaseKernel(kernel);
    clReleaseProgram(program);
}

void DescGradientMaker::Setup()
{
    SetupBase();
    sprintf(kernel_name, "%s/ClKernels/compute_gradient.cl", PCApps_PATH);
    program = BuildClProgram(device, context, kernel_name, include_path);
    kernel = CreateClkernel(program, "compute_gradient");

    descriptorData.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    szDescriptors = descriptorData.ByteSize();
    b_init = true;
}

void DescGradientMaker::ComputeGradient(cl_mem memPoints, cl_mem memNormals, cl_mem memNeighborIndices, cl_mem memNumNeighbors
                                          , cl_int maxNeighbors, cl_mem memDescriptors)
{
    if(b_init==false)
        Setup();
    cl_float4* descriptors = descriptorData.GetArrayPtr();
    cl_int status = 0;
    QElapsedTimer eltimer;

    // excute kernel
    eltimer.start();
    cl_event wlist[2];
    status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void*)&memNormals);
    status = clSetKernelArg(kernel, 2, sizeof(cl_mem), (void*)&memNeighborIndices);
    status = clSetKernelArg(kernel, 3, sizeof(cl_mem), (void*)&memNumNeighbors);
    status = clSetKernelArg(kernel, 4, sizeof(cl_int), (void*)&maxNeighbors);
    status = clSetKernelArg(kernel, 5, sizeof(cl_mem), (void*)&memDescriptors);
    status = clSetKernelArg(kernel, 6, sizeof(cl_mem), (void*)&memDebug);

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
    qDebug() << "   clEnqueueNDRangeKernel took" << eltimer.nsecsElapsed()/1000 << "us";

    // copy back output of kernel to host buffer
    eltimer.start();
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
                        queue,              // command queue
                        memDebug,           // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szDebug,            // size
                        debugBuffer,        // dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(debugBuffer)");
    qDebug() << "   clEnqueueReadBuffer took" << eltimer.nsecsElapsed()/1000 << "us";
}

cl_float4* DescGradientMaker::GetDescriptor()
{
    return descriptorData.GetArrayPtr();
}
