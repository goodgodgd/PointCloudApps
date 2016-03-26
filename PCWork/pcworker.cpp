#include "pcworker.h"

PCWorker::PCWorker()
{
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
    delete[] pointCloud;
    delete[] normalCloud;
    delete[] descriptorCloud;
    delete[] neighborIndices;
    delete[] numNeighbors;
}

void PCWorker::Work(QImage& srcColorImg, cl_float4* srcPointCloud)
{
    // copy input data
    colorImg = srcColorImg;
    memcpy(pointCloud, srcPointCloud, IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(cl_float4));

    const float searchRadius = 0.025f;
    const float forcalLength = 300.f;

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, searchRadius, forcalLength, NEIGHBORS_PER_POINT
                                      , neighborIndices, numNeighbors); // output
    qDebug() << "SearchNeighborIndices took" << eltimer.nsecsElapsed()/1000 << "us";
    int nbindex = neighborIndices[(150*IMAGE_WIDTH + 150)*NEIGHBORS_PER_POINT];
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 150] << nbindex << nbindex/IMAGE_WIDTH << nbindex%IMAGE_WIDTH
             << numNeighbors[150*IMAGE_WIDTH + 150];

    eltimer.start();
    normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT
                              , normalCloud); // output
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 150] << normalCloud[150*IMAGE_WIDTH + 150];

    eltimer.start();
    descriptorMaker.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT
                                      , descriptorCloud); // output
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 150] << descriptorCloud[150*IMAGE_WIDTH + 150];


    // point cloud segmentation
    // implement: (large) plane extraction, flood fill, segmentation based on (point distance > td || concave && color difference > tc)

    // descriptor clustering

    // descriptor matching

    // compute transformation

}

void PCWorker::DrawPointCloud(int viewOption)
{
    DrawUtils::DrawPointCloud(viewOption, pointCloud, normalCloud, colorImg, descriptorCloud);
}

void PCWorker::MarkNeighborsOnImage(QImage& srcimg, QPoint pixel)
{
    DrawUtils::MarkNeighborsOnImage(srcimg, pixel, neighborIndices, numNeighbors);
}

void PCWorker::MarkPoint3D(QPoint pixel, int viewOption)
{
    const int ptidx = IMGIDX(pixel.y(),pixel.x());
    DrawUtils::MarkPoint3D(viewOption, pointCloud[ptidx], normalCloud[ptidx], colorImg.pixel(pixel), descriptorCloud[ptidx]);
}

void PCWorker::DrawOnlyNeighbors(QPoint pixel)
{
    DrawUtils::DrawOnlyNeighbors(pixel, pointCloud, normalCloud, neighborIndices, numNeighbors, colorImg);
}
