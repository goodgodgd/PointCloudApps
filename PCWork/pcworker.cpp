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
    shdDat->SetColorImage(srcColorImg);
    colorImg = srcColorImg;

    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(srcDepthImg);
    shdDat->SetPointCloud(pointCloud);

    CreateNormalAndDescriptor(shdDat);

    eltimer.start();
    const cl_uchar* nullityMap = CreateNullityMap(shdDat);
    shdDat->SetNullityMap(nullityMap);
    qDebug() << "CreateNullityMap took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    planeClusterer.Cluster(shdDat);
    shdDat->SetPlaneMap(planeClusterer.GetSegmentMap());
    shdDat->SetPlanes(planeClusterer.GetSegments());
    qDebug() << "planeClusterer took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    planeMerger.ClusterPlanes(shdDat);
    shdDat->SetPlaneMap(planeMerger.GetObjectMap());
    shdDat->SetPlanes(planeMerger.GetObjects());
    qDebug() << "planeMerger took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    objectClusterer.ClusterPlanes(shdDat);
    shdDat->SetObjectMap(objectClusterer.GetObjectMap());
    shdDat->SetObjects(objectClusterer.GetObjects());
    qDebug() << "objectClusterer took" << eltimer.nsecsElapsed()/1000 << "us";

    // clustering into small blobs constrained by difference of point, normal and descriptor
    // blob size is constrained by 1/descriptor (larger blob is allowed for smaller descriptor)


    // descriptor matching

    // compute transformation

}

void PCWorker::CreateNormalAndDescriptor(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, SEARCH_RADIUS, FOCAL_LENGTH, NEIGHBORS_PER_POINT);
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborIndices took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices
                              , neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    const cl_float4* normalCloud = normalMaker.GetNormalCloud();
    shdDat->SetNormalCloud(normalCloud);
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";

//    normalSmoother.SmootheNormalCloud(pointCloud, normalCloud);
//    pointSmoother.SmoothePointCloud(pointCloud, normalCloud);

    eltimer.start();
    descriptorMaker.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    const cl_float4* descriptors = descriptorMaker.GetDescriptor();
    shdDat->SetDescriptors(descriptors);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";

//    CheckDataValidity(pointCloud, normalCloud, descriptors);
    shdDat->dataFilled = true;
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

cl_uchar* PCWorker::CreateNullityMap(SharedData* shdDat)
{
    static ArrayData<cl_uchar> nullData(IMAGE_HEIGHT*IMAGE_WIDTH);
    cl_uchar* nullityMap = nullData.GetArrayPtr();

    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const cl_float4* descriptors = shdDat->ConstDescriptors();

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
