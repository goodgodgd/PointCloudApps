#include "radiussearch.h"

RadiusSearch::RadiusSearch()
{
}

RadiusSearch::~RadiusSearch()
{
    clReleaseMemObject(memPoints);
    clReleaseMemObject(memNeighborIndices);
    clReleaseMemObject(memNumNeighbors);
    clReleaseKernel(kernel);
    clReleaseProgram(program);
}

void RadiusSearch::Setup()
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

    program = BuildClProgram(device, context, "../PCApps/ClKernels/search_neighbor_indices.cl", "-I../PCApps/ClKernels");
    kernel = CreateClkernel(program, "search_neighbor_indices");

    memPoints = CreateClImageFloat4(context, IMAGE_WIDTH, IMAGE_HEIGHT, CL_MEM_READ_ONLY);
    szNeighborIdcs = IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_int)*NEIGHBORS_PER_POINT;
    memNeighborIndices = CreateClBuffer(context, szNeighborIdcs, CL_MEM_READ_WRITE);
    szNumNeighbors = IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(cl_int);
    memNumNeighbors = CreateClBuffer(context, szNumNeighbors, CL_MEM_READ_WRITE);

    szDebug = DEBUG_FL_SIZE*sizeof(cl_float);
    memDebug = CreateClBuffer(context, szDebug, CL_MEM_WRITE_ONLY);

    b_init = true;
}

void RadiusSearch::SearchNeighborIndices(cl_float4* srcPointCloud, cl_float radiusMeter, cl_float focalLength, cl_int maxNeighbors
                          , cl_int* neighborIndices_out, cl_int* numNeighbors_out)
{
    if(b_init==false)
        Setup();

    cl_int status = 0;
    cl_event wlist[2];
    QElapsedTimer eltimer;

    // copy host buffer to input image object
    eltimer.start();
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
    qDebug() << "   clEnqueueWriteImage took" << eltimer.nsecsElapsed()/1000 << "us";

    // excute kernel
    eltimer.start();
    status = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void*)&memPoints);
    status = clSetKernelArg(kernel, 1, sizeof(cl_float), (void*)&radiusMeter);
    status = clSetKernelArg(kernel, 2, sizeof(cl_float), (void*)&focalLength);
    status = clSetKernelArg(kernel, 3, sizeof(cl_int), (void*)&maxNeighbors);
    status = clSetKernelArg(kernel, 4, sizeof(cl_mem), (void*)&memNeighborIndices);
    status = clSetKernelArg(kernel, 5, sizeof(cl_mem), (void*)&memNumNeighbors);
    status = clSetKernelArg(kernel, 6, sizeof(cl_mem), (void*)&memDebug);

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
    qDebug() << "   clEnqueueNDRangeKernel took" << eltimer.nsecsElapsed()/1000 << "us";

    // copy back output of kernel to host buffer
    eltimer.start();
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memNeighborIndices, // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szNeighborIdcs,     // size
                        neighborIndices_out,// dst host memory
                        0, NULL, NULL);     // events
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(memNeighborIndices)");
    status = clEnqueueReadBuffer(
                        queue,              // command queue
                        memNumNeighbors,    // device memory
                        CL_TRUE,            // block until finish
                        0,                  // offset
                        szNumNeighbors,     // size
                        numNeighbors_out,   // dst host memory
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
    LOG_OCL_ERROR(status, "clEnqueueReadBuffer(debugBuffer)");
    qDebug() << "   clEnqueueReadBuffer took" << eltimer.nsecsElapsed()/1000 << "us";
}


