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
    sprintf(kernel_name, "%s/ClKernels/compute_normal_vector.cl", PCApps_PATH);
    program = BuildClProgram(device, context, kernel_name, include_path);
    kernel = CreateClkernel(program, "compute_normal_vector");

    memNormals = CreateClImageFloat4(context, IMAGE_WIDTH, IMAGE_HEIGHT, CL_MEM_READ_WRITE);
    normalData.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    bInit = true;
}

void NormalMaker::ComputeNormal(cl_mem memPoints, cl_mem memNeighborIndices, cl_mem memNumNeighbors, cl_int maxNeighbors)
{
    if(bInit==false)
        Setup();
    cl_float4* normalCloud = normalData.GetArrayPtr();
    cl_int status = 0;
    float radius = NormalRadius();

    // excute kernel
    cl_event wlist[2];
    status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void*)&memNeighborIndices);
    status = clSetKernelArg(kernel, 2, sizeof(cl_mem), (void*)&memNumNeighbors);
    status = clSetKernelArg(kernel, 3, sizeof(cl_int), (void*)&maxNeighbors);
    status = clSetKernelArg(kernel, 4, sizeof(cl_int), (void*)&radius);
    status = clSetKernelArg(kernel, 5, sizeof(cl_mem), (void*)&memNormals);
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

//    {
//        QDebug dbg = qDebug();
//        dbg << "normal maker debug";
//        for(int i=0; i<30; ++i)
//            dbg << debugBuffer[i];
//    }
}

cl_float4* NormalMaker::GetNormalCloud()
{
    return normalData.GetArrayPtr();
}
