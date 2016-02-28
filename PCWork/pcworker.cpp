#include "pcworker.h"

PCWorker::PCWorker()
{
    // initialize CL memories and programs
    clworker = new CLWorker;

    // memory allocation
    pointCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];
    normalCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];
    descriptorCloud = new DescType[IMAGE_HEIGHT*IMAGE_WIDTH];
    neighborPoints = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH*NEIGHBORS_PER_POINT];
}

PCWorker::~PCWorker()
{
    // release memories
    delete clworker;
    delete[] pointCloud;
    delete[] normalCloud;
    delete[] descriptorCloud;
    delete[] neighborPoints;
}

void PCWorker::SetInputs(QImage& srcColorImg, cl_float4* srcPointCloud, int inViewOption)
{
    colorImg = srcColorImg;
    memcpy(pointCloud, srcPointCloud, IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(cl_float4));
    viewOption = inViewOption;
}

void PCWorker::Work()
{
    const float normalRadius = 0.02f;
    const float forcalLength = 300.f;
    const float descriptorRadius = 0.03f;

    eltimer.start();
    clworker->SearchNeighborPoints(pointCloud, normalRadius, forcalLength, NEIGHBORS_PER_POINT
                                , neighborPoints);  // output
    qDebug() << "SearchNeighborPoints took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 200] << neighborPoints[(150*IMAGE_WIDTH + 200)*NEIGHBORS_PER_POINT];

    eltimer.start();
    clworker->ComputeNormalWithPointGroups(NULL, NEIGHBORS_PER_POINT
                                        , normalCloud);  // output
    qDebug() << "ComputeNormalWithPointGroups took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 200] << normalCloud[150*IMAGE_WIDTH + 200];

    /*
    // compute normal vectors of point cloud using opencl
    eltimer.start();
    clworker->ComputeNormal(pointCloud, normalRadius, forcalLength
                            , normalCloud);         // output
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "kernel output" << pointCloud[150*IMAGE_WIDTH + 200] << normalCloud[150*IMAGE_WIDTH + 200];

    // point cloud segmentation
    // implement: (large) plane extraction, flood fill, segmentation based on (point distance > td || concav456e && color difference > tc)

    // compute descriptors of point cloud using opencl
    eltimer.start();
    clworker->ComputeDescriptor(NULL, NULL, descriptorRadius, forcalLength
                                , descriptorCloud); // output
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";
*/

    // descriptor clustering

    // descriptor matching

    // compute transformation


    if(viewOption & ViewOpt::WholeCloud)
        DrawPointCloud(pointCloud, normalCloud, viewOption);
}

void PCWorker::DrawPointCloud(cl_float4* pointCloud, cl_float4* normalCloud, int viewOption)
{
    // point color: white
    cl_float4 ptcolor = cl_float4{1,1,1,1};
    const float normalLength = 0.02f;
    QRgb pixelColor;
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

        // add point vertex
        gvm::AddVertex(eVertexType::point, pointCloud[i], ptcolor, normalCloud[i], 1);

        // add line vertices
        if(viewOption & ViewOpt::WCNormal)
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






