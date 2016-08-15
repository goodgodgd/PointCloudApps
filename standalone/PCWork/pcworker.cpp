#include "pcworker.h"

PCWorker::PCWorker()
    : neighborIndices(nullptr)
    , numNeighbors(nullptr)
{
}

PCWorker::~PCWorker()
{
}

void PCWorker::Work(const QImage& srcColorImg, const QImage& srcDepthImg, const Pose6dof& framePose, SharedData* shdDat/*, vector<AnnotRect>& annotRects*/)
{
    shdDat->SetColorImage(srcColorImg);
    colorImg = srcColorImg;

    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(srcDepthImg);
    shdDat->SetPointCloud(pointCloud);

    SearchNeighborsAndCreateNormal(shdDat);

    ComputeDescriptorsCpu(shdDat);
//    ComputeDescriptorsGpu(shdDat);

    const cl_uchar* nullityMap = CreateNullityMap(shdDat);
    shdDat->SetNullityMap(nullityMap);

    ClusterPointsOfObjects(shdDat);
}

void PCWorker::SearchNeighborsAndCreateNormal(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, DESCRIPTOR_RADIUS, CameraParam::flh(), MAX_NEIGHBORS);
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborIndices took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices
                              , neibSearcher.memNumNeighbors, MAX_NEIGHBORS);
    const cl_float4* normalCloud = normalMaker.GetNormalCloud();
    shdDat->SetNormalCloud(normalCloud);
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";
}

void PCWorker::ComputeDescriptorsCpu(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const DescType* descriptors = nullptr;
    const AxesType* descAxes = nullptr;

    eltimer.start();
    descriptorMakerCpu.ComputeDescriptors(pointCloud, normalCloud, neighborIndices, numNeighbors, MAX_NEIGHBORS, DESCRIPTOR_RADIUS);
    descriptors = descriptorMakerCpu.GetDescriptors();
    descAxes = descriptorMakerCpu.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetDescAxes(descAxes);
    qDebug() << "ComputeDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";

//    eltimer.start();
//    gradientMakerCpu.CopmuteGradient(pointCloud, descriptors, descAxes
//                                     , neighborIndices, numNeighbors, MAX_NEIGHBORS, DESCRIPTOR_RADIUS);
//    descriptors = gradientMakerCpu.GetGradDesc();
//    shdDat->SetDescriptors(descriptors);
//    qDebug() << "ComputeDescGradientCpu took" << eltimer.nsecsElapsed()/1000 << "us";

    // ========== check validity ==========
    eltimer.start();
    descriptorMaker.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, MAX_NEIGHBORS);
    const DescType* descriptorsGpu = descriptorMaker.GetDescriptor();
    const AxesType* descAxesGpu = descriptorMaker.GetDescAxes();
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";

    CheckDataValidity(shdDat, descriptorsGpu, descAxesGpu);
}

void PCWorker::ComputeDescriptorsGpu(SharedData* shdDat)
{
    eltimer.start();
    descriptorMaker.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, MAX_NEIGHBORS);
    const DescType* descriptors = descriptorMaker.GetDescriptor();
    shdDat->SetDescriptors(descriptors);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
//    gradientMaker.ComputeGradient(neibSearcher.memPoints, descriptorMaker.memDescriptors, descriptorMaker.memDescAxes
//                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, MAX_NEIGHBORS);
//    descriptors = gradientMaker.GetGradDesc();
//    shdDat->SetDescriptors(descriptors);
    qDebug() << "ComputeDescGradient took" << eltimer.nsecsElapsed()/1000 << "us";
}

void PCWorker::ClusterPointsOfObjects(SharedData* shdDat)
{
    eltimer.start();
    planeClusterer.Cluster(shdDat);
    shdDat->SetPlaneMap(planeClusterer.GetSegmentMap());
    shdDat->SetPlanes(planeClusterer.GetSegments());
    qDebug() << "planeClusterer took" << eltimer.nsecsElapsed()/1000 << "us";

//    eltimer.start();
//    planeMerger.ClusterPlanes(shdDat);
//    shdDat->SetPlaneMap(planeMerger.GetObjectMap());
//    shdDat->SetPlanes(planeMerger.GetObjects());
//    qDebug() << "planeMerger took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    objectClusterer.ClusterPlanes(shdDat);
    shdDat->SetObjectMap(objectClusterer.GetObjectMap());
    shdDat->SetObjects(objectClusterer.GetObjects());
    qDebug() << "objectClusterer took" << eltimer.nsecsElapsed()/1000 << "us";
}

void PCWorker::CheckDataValidity(SharedData* shdDat, const cl_float4* descriptorsGpu, const AxesType* descAxesGpu)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const DescType* descriptors = shdDat->ConstDescriptors();
    const AxesType* descAxes = shdDat->ConstDescAxes();
    cl_float4 axes[2];

    // 1. w channel of point cloud, normal cloud and descriptors must be "0"
    // 2. length of normal is either 0 or 1
    // 3. w channel of descriptors is "1" if valid, otherwise "0"
    int count=0;
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        assert(pointCloud[i].w==0.f);
        assert(normalCloud[i].w==0.f);
        assert(clIsNormalized(normalCloud[i]) || clIsNull(normalCloud[i]));

        clSplit(descAxes[i], axes);
        assert(clIsNormalized(axes[0]) || clIsNull(axes[0]));
        assert(clIsNormalized(axes[1]) || clIsNull(axes[1]));

        if(descriptorsGpu!=nullptr)
            if(fabsf(descriptors[i].x - descriptorsGpu[i].x) > 0.001f || fabsf(descriptors[i].y - descriptorsGpu[i].y) > 0.001f)
                count++;
        if(descAxesGpu!=nullptr)
        {
            if(fabsf(descAxes[i].s[0] - descAxesGpu[i].s[0]) > 0.001f)
                count++;
            if(fabsf(descAxes[i].s[4] - descAxesGpu[i].s[4]) > 0.001f)
                count++;
            if(fabsf(descAxes[i].s[3]) > 0.001f)
                count++;
            if(fabsf(descAxes[i].s[7]) > 0.001f)
                count++;
        }
    }
    if(count > 100)
        TryFrameException(QString("too many disagrees %1").arg(count));
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
