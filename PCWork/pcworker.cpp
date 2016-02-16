#include "pcworker.h"

PCWorker::PCWorker()
{
    // initialize CL memories and programs
    m_clworker = new CLWorker;

    // memory allocation
    m_pointCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];
    m_normalCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];
    m_qvPointCloud = new QVector3D*[IMAGE_HEIGHT];
    for(int y=0; y<IMAGE_HEIGHT; y++)
        m_qvPointCloud[y] = new QVector3D[IMAGE_WIDTH];
}

PCWorker::~PCWorker()
{
    // release memories
    delete m_clworker;
    delete[] m_pointCloud;
    delete[] m_normalCloud;
    for(int y=0; y<IMAGE_HEIGHT; y++)
        delete[] m_qvPointCloud[y];
    delete[] m_qvPointCloud;
}

void PCWorker::SetInputs(QImage& colorImg, cl_float4* srcPointCloud)
{
    // copy input data
    m_colorImg = colorImg;
    
    // copy point cloud
    memcpy(m_pointCloud, srcPointCloud, IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(cl_float4));
#ifdef VIZ3D
#pragma omp parallel for
    for(int y=0; y<IMAGE_HEIGHT; y++)
        for(int x=0; x<IMAGE_WIDTH; x++)
            m_qvPointCloud[y][x] << srcPointCloud[y*IMAGE_WIDTH + x];
#endif // VIZ3D
}

void PCWorker::Work()
{
    // compute normal vectors of point cloud using opencl
    m_clworker->ComputeNormal(m_pointCloud, 0.1f, 300.f, m_normalCloud);
    qDebug() << "kernel output" << m_pointCloud[150*IMAGE_WIDTH + 200] << m_normalCloud[150*IMAGE_WIDTH + 200];

    // point cloud segmentation
    // implement: (large) plane extraction, flood fill, segmentation based on (point distance > td || concav456e && color difference > tc)

    // compute descriptors of point cloud using opencl

    // descriptor clustering

    // descriptor matching

    // compute transformation


    //DrawPoints(m_pointCloud);
}
















