#include "pcworker.h"

PCWorker::PCWorker()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , descriptors(nullptr)
    , nullityMap(nullptr)
    , planeMap(nullptr)
{
}

PCWorker::~PCWorker()
{
}

void PCWorker::Work(SharedData* shdDat, const QImage& srcColorImg, const QImage& srcDepthImg)
{
    const float searchRadius = 0.02f;
    const float forcalLength = 300.f;
    shdDat->SetColorImage(srcColorImg);
    colorImg = srcColorImg;

    pointCloud = ImageConverter::ConvertToPointCloud(srcDepthImg);
    shdDat->SetPointCloud(pointCloud);

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, searchRadius, forcalLength, NEIGHBORS_PER_POINT);
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborIndices took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    normalCloud = normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices
                                            , neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    shdDat->SetNormalCloud(normalCloud);
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";

//    normalSmoother.SmootheNormalCloud(pointCloud, normalCloud);
//    pointSmoother.SmoothePointCloud(pointCloud, normalCloud);

    eltimer.start();
    descriptors = descriptorMaker.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                                    , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    shdDat->SetDescriptors(descriptors);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";

    CheckDataValidity();

    eltimer.start();
    nullityMap = CreateNullityMap();
    shdDat->SetNullityMap(nullityMap);
    qDebug() << "CreateNullityMap took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    planeMap = planeClusterer.Cluster(shdDat, nullptr);
    shdDat->SetSegmentMap(planeMap);
    qDebug() << "planeClusterer took" << eltimer.nsecsElapsed()/1000 << "us";


    // point cloud segmentation
    // implement: (large) plane extraction, flood fill, segmentation based on (point distance > td || concave && color difference > tc)

    // descriptor clustering

    // descriptor matching

    // compute transformation

    shdDat->dataFilled = true;
}

void PCWorker::CheckDataValidity()
{
    // 1. w channel of point cloud, normal cloud and descriptors must be "0"
    // 2. length of normal is either 0 or 1
    // 3. w channel of descriptors is "1" if valid, otherwise "0"
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        assert(pointCloud[i].w==0.f);
        assert(normalCloud[i].w==0.f);
        assert(fabsf(clLength(normalCloud[i])-1.f) < 0.0001f || clLength(normalCloud[i]) < 0.0001f);
        assert(descriptors[i].w==0.f);
    }
}

cl_uchar* PCWorker::CreateNullityMap()
{
    static ArrayData<cl_uchar> nullData(IMAGE_HEIGHT*IMAGE_WIDTH);
    cl_uchar* nullityMap = nullData.GetArrayPtr();


    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        nullityMap[i] = NullID::NoneNull;
        if(clIsNull(pointCloud[i]))
            nullityMap[i] = NullID::PointNull;
        else if(clIsNull(normalCloud[i]))
            nullityMap[i] = NullID::NormalNull;
        else if(clIsNull(descriptors[i]))           // must be updated!!
            nullityMap[i] = NullID::DescriptorNull;
    }
    return nullityMap;
}


// 1. move draw functions to DrawUtils
// 2. add segmentmap to shared data
// 3. remove class variables from shared data

void PCWorker::DrawPointCloud(int viewOption)
{
    if(viewOption == ViewOpt::ViewNone)
        return;

    if(viewOption & ViewOpt::Color)
        DrawUtils::SetColorMapByRgbImage(colorImg);
    else if(viewOption & ViewOpt::Descriptor)
        DrawUtils::SetColorMapByDescriptor(descriptors, nullityMap);
    else if(viewOption & ViewOpt::Segment)
        DrawUtils::SetColorMapByCluster(planeMap);
//    else if(viewOption & ViewOpt::Object)
//        DrawUtils::SetColorMapByCluster(planeClusterer.segmap);

    DrawUtils::DrawPointCloud(pointCloud, normalCloud);
    if(viewOption | ViewOpt::Normal)
        DrawUtils::DrawNormalCloud(pointCloud, normalCloud);
}

void PCWorker::MarkNeighborsOnImage(QImage& srcimg, QPoint pixel)
{
    DrawUtils::MarkNeighborsOnImage(srcimg, pixel, neighborIndices, numNeighbors);
}

void PCWorker::MarkPoint3D(QPoint pixel)
{
    const int ptidx = IMGIDX(pixel.y(),pixel.x());
    DrawUtils::MarkPoint3D(pointCloud[ptidx], normalCloud[ptidx], colorImg.pixel(pixel));
}

void PCWorker::DrawOnlyNeighbors(QPoint pixel, int viewOption)
{
    DrawUtils::DrawOnlyNeighbors(pixel, pointCloud, normalCloud, neighborIndices, numNeighbors, colorImg);
}
