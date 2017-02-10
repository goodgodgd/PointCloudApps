#include "radiussearch.h"

RadiusSearch::RadiusSearch()
{
}

RadiusSearch::~RadiusSearch()
{
    clReleaseMemObject(memPoints);
    clReleaseMemObject(memNeighborIndices);
    clReleaseMemObject(memNumNeighbors);
    clReleaseMemObject(memDebug);
    clReleaseKernel(kernel);
    clReleaseProgram(program);
}

void RadiusSearch::Setup()
{
    SetupBase();
    sprintf(kernel_name, "%s/ClKernels/search_neighbor_indices.cl", PCApps_PATH);
    program = BuildClProgram(device, context, kernel_name, include_path);
    kernel = CreateClkernel(program, "search_neighbor_indices");

    memPoints = CreateClImageFloat4(context, IMAGE_WIDTH, IMAGE_HEIGHT, CL_MEM_READ_ONLY);

    neibIndicesData.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT*MaxNeighbors());
    szNeighborIdcs = neibIndicesData.ByteSize();
    memNeighborIndices = CreateClBuffer(context, szNeighborIdcs, CL_MEM_READ_WRITE);
    qDebug() << "Neighbors array size" << neibIndicesData.ArraySize();

    numNeibsData.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    szNumNeighbors = numNeibsData.ByteSize();
    memNumNeighbors = CreateClBuffer(context, szNumNeighbors, CL_MEM_READ_WRITE);
    bInit = true;
}

void RadiusSearch::SearchNeighborIndices(const cl_float4* srcPointCloud, cl_float radiusMeter, cl_int neighborLimit, cl_float focalLength)
{
    if(bInit==false)
        Setup();
    const int maxNeighbors = MaxNeighbors();
    if(maxNeighbors < neighborLimit)
        throw TryFrameException("insufficient max neighbors");

    cl_int* neighborIndices = neibIndicesData.GetArrayPtr();
    cl_int* numNeighbors = numNeibsData.GetArrayPtr();
    memset(debugBuffer, 0x00, sizeof(cl_float)*DEBUG_FL_SIZE);

    cl_int status = 0;
    cl_event wlist[2];
    QElapsedTimer eltimer;

    // copy host buffer to input image object
//    eltimer.start();
    status = clEnqueueWriteImage(
                        queue,              // command queue
                        memPoints,          // device memory
                        CL_TRUE,            // block until finish
                        imgOrigin,          // imgOrigin of image
                        imgRegion,          // imgRegion of image
                        0, 0,               // row pitch, slice pitch
                        (void*)srcPointCloud,  // source host memory
                        0, NULL, NULL);     // wait, event
    LOG_OCL_ERROR(status, "clEnqueueWriteImage(memPoints)" );
//    qDebug() << "   clEnqueueWriteImage took" << eltimer.nsecsElapsed()/1000 << "us";

    // excute kernel
//    eltimer.start();
    status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(kernel, 1, sizeof(cl_float), (void*)&radiusMeter);
    status = clSetKernelArg(kernel, 2, sizeof(cl_float), (void*)&focalLength);
    status = clSetKernelArg(kernel, 3, sizeof(cl_int), (void*)&neighborLimit);
    status = clSetKernelArg(kernel, 4, sizeof(cl_mem), (void*)&memNeighborIndices);
    status = clSetKernelArg(kernel, 5, sizeof(cl_mem), (void*)&memNumNeighbors);
    status = clSetKernelArg(kernel, 6, sizeof(cl_int), (void*)&maxNeighbors);
    status = clSetKernelArg(kernel, 7, sizeof(cl_mem), (void*)&memDebug);

    status = clEnqueueNDRangeKernel(
                        queue,              // command queue
                        kernel,             // kernel
                        2,                  // dimension
                        NULL,               // global offset
                        gwsize,             // global work size
                        lwsize,             // local work size
                        0,                  // # of wait lists
                        NULL,               // wait list
                        &wlist[0]);         // event output
    LOG_OCL_ERROR(status, "clEnqueueNDRangeKernel(kernel)");
    clWaitForEvents(1, &wlist[0]);
//    qDebug() << "   clEnqueueNDRangeKernel took" << eltimer.nsecsElapsed()/1000 << "us";

    // copy back output of kernel to host buffer
//    eltimer.start();
    /*
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memNeighborIndices, // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szNeighborIdcs,     // size
                        neighborIndices,// dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memNeighborIndices)");
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memNumNeighbors,    // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szNumNeighbors,     // size
                        numNeighbors,   // dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memNumNeighbors)");
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memDebug,           // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szDebug,            // size
                        debugBuffer,        // dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memDebug)");
//    qDebug() << "   clEnqueueReadBuffer took" << eltimer.nsecsElapsed()/1000 << "us";
    */

//    int numDbg = debugBuffer[0];
//    {
//        QDebug dbg = qDebug();
//        dbg << "numdebug" << numDbg;
//        for(int i=1; i<numDbg; i++)
//            dbg << debugBuffer[i];
//    }
}

cl_int* RadiusSearch::GetNeighborIndices()
{
    return neibIndicesData.GetArrayPtr();
}

cl_int* RadiusSearch::GetNumNeighbors()
{
    return numNeibsData.GetArrayPtr();
}
