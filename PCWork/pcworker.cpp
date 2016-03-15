#include "pcworker.h"

PCWorker::PCWorker()
{
    // initialize CL memories and programs
    clworker = new CLWorker;

    // memory allocation
    pointCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];
    normalCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];
    descriptorCloud = new DescType[IMAGE_HEIGHT*IMAGE_WIDTH];
    neighborIndices = new cl_int[IMAGE_HEIGHT*IMAGE_WIDTH*NEIGHBORS_PER_POINT];
    numNeighbors = new cl_int[IMAGE_HEIGHT*IMAGE_WIDTH];
}

PCWorker::~PCWorker()
{
    // release memories
    delete clworker;
    delete[] pointCloud;
    delete[] normalCloud;
    delete[] descriptorCloud;
    delete[] neighborIndices;
    delete[] numNeighbors;
}

void PCWorker::SetInputs(QImage& srcColorImg, cl_float4* srcPointCloud, int inViewOption)
{
    colorImg = srcColorImg;
    memcpy(pointCloud, srcPointCloud, IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(cl_float4));
    viewOption = inViewOption;
}

void PCWorker::Work()
{
//    TestSolver();
//    return;

    const float searchRadius = 0.02f;
    const float forcalLength = 300.f;

    eltimer.start();
    clworker->SearchNeighborPoints(pointCloud, searchRadius, forcalLength
                                   , neighborIndices, numNeighbors);  // output
    qDebug() << "SearchNeighborPoints took" << eltimer.nsecsElapsed()/1000 << "us";
    int nbindex = neighborIndices[(150*IMAGE_WIDTH + 150)*NEIGHBORS_PER_POINT];
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 150] << nbindex << nbindex/IMAGE_WIDTH << nbindex%IMAGE_WIDTH
             << numNeighbors[150*IMAGE_WIDTH + 150];

    eltimer.start();
    clworker->ComputeNormalWithNeighborPts(normalCloud);
    qDebug() << "ComputeNormalWithNeighborPts took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 150] << normalCloud[150*IMAGE_WIDTH + 150];
    CheckNaN(normalCloud);

    shapeDesc.ComputeDescriptorCloud(pointCloud, normalCloud, neighborIndices, numNeighbors, NEIGHBORS_PER_POINT
                                     , descriptorCloud);     // output
    qDebug() << "ComputeDescriptorByCPU" << descriptorCloud[150*IMAGE_WIDTH + 150];

    eltimer.start();
    clworker->ComputeDescriptorWithNeighborPts(descriptorCloud);
    qDebug() << "ComputeDescriptorWithNeighborPts took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 150] << descriptorCloud[150*IMAGE_WIDTH + 150];


    // point cloud segmentation
    // implement: (large) plane extraction, flood fill, segmentation based on (point distance > td || concave && color difference > tc)

    // descriptor clustering

    // descriptor matching

    // compute transformation


    if(viewOption != ViewOpt::ViewNone)
        DrawPointCloud(viewOption);
}

void PCWorker::DrawPointCloud(int viewOption)
{
    // point color: white
    cl_float4 ptcolor = cl_float4{1,1,1,1};
    const float normalLength = 0.02f;
    QRgb pixelColor;
    cl_float4 descrColor;
    int x, y;
    ptcolor = clNormalize(ptcolor);

    // add point cloud with size of 2
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        if(clIsNull(pointCloud[i]) || clIsNull(normalCloud[i]))
            continue;

        y = i/IMAGE_WIDTH;
        x = i%IMAGE_WIDTH;

        // set point color from color image
        if(viewOption & ViewOpt::WCColor)
        {
            pixelColor = colorImg.pixel(x, y);
            ptcolor << pixelColor;
        }
        else if(viewOption & ViewOpt::WCDescriptor)
        {
            ptcolor = ConvertDescriptorToColor(descriptorCloud[i]);
        }

        // add point vertex
        gvm::AddVertex(eVertexType::point, pointCloud[i], ptcolor, normalCloud[i], 1);

        // add line vertices
        if(viewOption & ViewOpt::Normal)
        {
            cl_float4 normalTip = pointCloud[i] + normalCloud[i] * normalLength;
            if(y%5==2 && x%5==2)
            {
                gvm::AddVertex(eVertexType::line, pointCloud[i], ptcolor, normalCloud[i], 1);
                gvm::AddVertex(eVertexType::line, normalTip, ptcolor, normalCloud[i], 1, true);
            }
        }
    }
}

cl_float4 PCWorker::ConvertDescriptorToColor(cl_float4 descriptor)
{
    const float color_range = 14.f;
    cl_float4 color;
    color.x = descriptor.x / color_range + 0.5f;
    color.x = smin(smax(color.x, 0.f), 1.f);
    color.y = descriptor.y / color_range + 0.5f;
    color.y = smin(smax(color.y, 0.f), 1.f);
    color.z = (2.f - color.x - color.y) / 2.f;
    color.z = smin(smax(color.z, 0.f), 1.f);
    return color;
}

void PCWorker::CheckNaN(cl_float4* points)
{
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            if(isnanf(points[y*IMAGE_WIDTH+x].x) || isnanf(points[y*IMAGE_WIDTH+x].y))
                qDebug() << "Normal NaN!!" << x << y << points[y*IMAGE_WIDTH+x].x;
        }
    }
}






