#include "normalmaker.h"

NormalMaker::NormalMaker()
{
}

NormalMaker::~NormalMaker()
{
    clReleaseMemObject(memNormals);
    clReleaseKernel(kernel);
    clReleaseProgram(program);
}

void NormalMaker::Setup()
{
    SetupBase();
    program = BuildClProgram(device, context, "../PCApps/ClKernels/compute_normal_vector.cl", "-I../PCApps/ClKernels");
    kernel = CreateClkernel(program, "compute_normal_vector");

    memNormals = CreateClImageFloat4(context, IMAGE_WIDTH, IMAGE_HEIGHT, CL_MEM_READ_WRITE);
    normalData.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    b_init = true;
}

cl_float4* NormalMaker::ComputeNormal(cl_mem memPoints, cl_mem memNeighborIndices, cl_mem memNumNeighbors, cl_int maxNeighbors)
{
    if(b_init==false)
        Setup();
    cl_float4* normalCloud = normalData.GetArrayPtr();
    cl_int status = 0;
    QElapsedTimer eltimer;

    eltimer.start();
    // excute kernel
    cl_event wlist[2];
    status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void*)&memNeighborIndices);
    status = clSetKernelArg(kernel, 2, sizeof(cl_mem), (void*)&memNumNeighbors);
    status = clSetKernelArg(kernel, 3, sizeof(cl_int), (void*)&maxNeighbors);
    status = clSetKernelArg(kernel, 4, sizeof(cl_mem), (void*)&memNormals);
    status = clSetKernelArg(kernel, 5, sizeof(cl_mem), (void*)&memDebug);

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

    eltimer.start();
    // copy back output of kernel to host buffer
    status = clEnqueueReadImage(
                        queue,              // command queue
                        memNormals,         // source device memory
                        CL_TRUE,            // block until finish
                        imgOrigin,          // imgOrigin of image
                        imgRegion,          // imgRegion of image
                        0, 0,               // row pitch, slice pitch
                        (void*)normalCloud, // host memory
                        0, NULL, NULL);     // wait, event
    LOG_OCL_ERROR(status, "clEnqueueReadImage(memNormals)");
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memDebug,           // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szDebug,            // size
                        debugBuffer,        // dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(debugBuffer)");
    qDebug() << "   clEnqueueReadImage took" << eltimer.nsecsElapsed()/1000 << "us";

    return normalCloud;
}
