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
    gwsize[0] = IMAGE_WIDTH;
    gwsize[1] = IMAGE_HEIGHT;
    lwsize[0] = 16;
    lwsize[1] = 16;
    imgOrigin[0] = imgOrigin[1] = imgOrigin[2] = 0;
    imgRegion[0] = IMAGE_WIDTH;
    imgRegion[1] = IMAGE_HEIGHT;
    imgRegion[2] = 1;

    device = ClSetup::GetDevice();
    context = ClSetup::GetContext();
    queue = ClSetup::GetQueue();

    program = BuildClProgram(device, context, "../PointCloudApps/ClKernels/compute_descriptor.cl", "-I../PointCloudApps/ClKernels");
    kernel = CreateClkernel(program, "compute_descriptor");

    szDescriptors = IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_float4);
    memDescriptors = CreateClBuffer(context, szDescriptors, CL_MEM_READ_WRITE);

    szDebug = DEBUG_FL_SIZE*sizeof(cl_float);
    memDebug = CreateClBuffer(context, szDebug, CL_MEM_WRITE_ONLY);

    b_init = true;
}

void DescriptorMaker::ComputeDescriptor(cl_mem memPoints, cl_mem memNormals, cl_mem memNeighborIndices, cl_mem memNumNeighbors, cl_int maxNeighbors
                                        , DescType* descriptorCloud_out)
{
    if(b_init==false)
        Setup();

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
                        descriptorCloud_out,   // dst host memory
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
