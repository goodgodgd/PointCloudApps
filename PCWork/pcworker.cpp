#include "pcworker.h"

PCWorker::PCWorker()
    : neighborIndices(nullptr)
    , numNeighbors(nullptr)
{
}

PCWorker::~PCWorker()
{
}

void PCWorker::Work(const QImage& srcColorImg, const QImage& srcDepthImg, SharedData* shdDat)
{
    const float searchRadius = 0.02f;
    const float forcalLength = 300.f;
    shdDat->SetColorImage(srcColorImg);
    colorImg = srcColorImg;

    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(srcDepthImg);
    shdDat->SetPointCloud(pointCloud);

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, searchRadius, forcalLength, NEIGHBORS_PER_POINT);
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborIndices took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    const cl_float4* normalCloud = normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices
                                            , neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    shdDat->SetNormalCloud(normalCloud);
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";

//    normalSmoother.SmootheNormalCloud(pointCloud, normalCloud);
//    pointSmoother.SmoothePointCloud(pointCloud, normalCloud);

    eltimer.start();
    const cl_float4* descriptors = descriptorMaker.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                            , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    shdDat->SetDescriptors(descriptors);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";

    CheckDataValidity(pointCloud, normalCloud, descriptors);
    shdDat->dataFilled = true;

    eltimer.start();
    const cl_uchar* nullityMap = CreateNullityMap(pointCloud, normalCloud, descriptors);
    shdDat->SetNullityMap(nullityMap);
    qDebug() << "CreateNullityMap took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    planeClusterer.Cluster(shdDat);
    const cl_int* planeMap = planeClusterer.GetSegmentMap();
    const vecSegment* planes = planeClusterer.GetSegments();
    shdDat->SetPlaneMap(planeMap);
    shdDat->SetPlanes(planes);
    qDebug() << "planeClusterer took" << eltimer.nsecsElapsed()/1000 << "us";

//    return;
    eltimer.start();
    objectCluster.ClusterCloudIntoObjects(shdDat);
    const cl_int* objectMap = objectCluster.GetObjectMap();
    const vecSegment* objects = objectCluster.GetObjects();
    shdDat->SetObjectMap(objectMap);
    shdDat->SetObjects(objects);
    qDebug() << "objectCluster took" << eltimer.nsecsElapsed()/1000 << "us";

    // point cloud segmentation
    // implement: (large) plane extraction, flood fill, segmentation based on (point distance > td || concave && color difference > tc)

    // descriptor clustering

    // descriptor matching

    // compute transformation

}

void PCWorker::CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud, const cl_float4* descriptors)
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

cl_uchar* PCWorker::CreateNullityMap(const cl_float4* pointCloud, const cl_float4* normalCloud, const cl_float4* descriptors)
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

void PCWorker::MarkNeighborsOnImage(QImage& srcimg, QPoint pixel)
{
    DrawUtils::MarkNeighborsOnImage(srcimg, pixel, neighborIndices, numNeighbors);
}

void PCWorker::DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel)
{
    DrawUtils::DrawOnlyNeighbors(pixel, shdDat.ConstPointCloud(), shdDat.ConstNormalCloud(), neighborIndices, numNeighbors, colorImg);
}
