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

//    ComputeDescriptorsCpu(shdDat);
    ComputeDescriptorsGpu(shdDat);

    const cl_uchar* nullityMap = CreateNullityMap(shdDat);
    shdDat->SetNullityMap(nullityMap);

    ClusterPointsOfObjects(shdDat);
}

void PCWorker::SearchNeighborsAndCreateNormal(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, NormalMaker::NormalRadius(), NormalMaker::NormalNeighbors(), CameraParam::flh());
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborForNormal took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, RadiusSearch::MaxNeighbors());
    const cl_float4* normalCloud = normalMaker.GetNormalCloud();
    shdDat->SetNormalCloud(normalCloud);
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";
}

void PCWorker::ComputeDescriptorsCpu(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const DescType* descriptors = nullptr;
    const AxesType* prinAxes = nullptr;

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, DescriptorMakerByCpu::DescriptorRadius()
                                       , DescriptorMakerByCpu::DescriptorNeighbors(), CameraParam::flh());
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborsForDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    descriptorMakerCpu.ComputeDescriptors(pointCloud, normalCloud, neighborIndices, numNeighbors, RadiusSearch::MaxNeighbors());
    descriptors = descriptorMakerCpu.GetDescriptors();
    prinAxes = descriptorMakerCpu.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetDescAxes(prinAxes);
    qDebug() << "ComputeDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "descriptor cpu" << descriptors[IMGIDX(100,100)] << prinAxes[IMGIDX(100,100)];

    // ========== check validity ==========
    ComputeDescriptorsGpu(shdDat);

    CheckDataValidity(shdDat, descriptors, prinAxes);
}

void PCWorker::ComputeDescriptorsGpu(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, DescriptorMakerByCpu::DescriptorRadius()
                                       , DescriptorMakerByCpu::DescriptorNeighbors(), CameraParam::flh());
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborsForDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    descriptorMaker.ComputeDescriptors(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, RadiusSearch::MaxNeighbors());
    const DescType* descriptors = descriptorMaker.GetDescriptor();
    const AxesType* prinAxes = descriptorMaker.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetDescAxes(prinAxes);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "descriptor gpu" << descriptors[IMGIDX(100,100)] << prinAxes[IMGIDX(100,100)];
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

void PCWorker::CheckDataValidity(SharedData* shdDat, const cl_float4* descriptorsGpu, const AxesType* prinAxesGpu)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const DescType* descriptors = shdDat->ConstDescriptors();
    const AxesType* prinAxes = shdDat->ConstPrinAxes();
    cl_float4 majorAxis, minorAxis;

    // 1. w channel of point cloud, normal cloud and descriptors must be "0"
    // 2. length of normal is either 0 or 1
    // 3. w channel of descriptors is "1" if valid, otherwise "0"
    int count=0;
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        assert(pointCloud[i].w==0.f);
        assert(normalCloud[i].w==0.f);
        assert(clIsNormalized(normalCloud[i]) || clIsNull(normalCloud[i]));

        clSplit(prinAxes[i], majorAxis, minorAxis);
        assert(clIsNormalized(majorAxis) || clIsNull(majorAxis));
        assert(clIsNormalized(minorAxis) || clIsNull(minorAxis));

        if(descriptorsGpu!=nullptr)
            if(clLength(descriptors[i] - descriptorsGpu[i]) > 0.001f)
                count++;
        if(prinAxesGpu!=nullptr)
        {
            if(fabsf(prinAxes[i].s[0] - prinAxesGpu[i].s[0]) > 0.001f)
                count++;
            if(fabsf(prinAxes[i].s[4] - prinAxesGpu[i].s[4]) > 0.001f)
                count++;
            if(fabsf(prinAxes[i].s[3]) > 0.001f)
                count++;
            if(fabsf(prinAxes[i].s[7]) > 0.001f)
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
    int nanCount=0, nullCount=0;

    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        nullityMap[i] = NullID::NoneNull;
        if(clIsNull(pointCloud[i]))
            nullityMap[i] = NullID::PointNull;
        else if(clIsNull(normalCloud[i]))
            nullityMap[i] = NullID::NormalNull;
        else if(clIsNull(descriptors[i]))           // must be updated!!
            nullityMap[i] = NullID::DescriptorNull;

        if(clIsNan(descriptors[i]))
            ++nanCount;
        if(clIsNull(descriptors[i]))
            ++nullCount;
    }
    qDebug() << "nan descriptors" << nanCount << nullCount;
    return nullityMap;
}

void PCWorker::MarkNeighborsOnImage(QImage& srcimg, QPoint pixel)
{
    DrawUtils::MarkNeighborsOnImage(srcimg, pixel, neighborIndices, numNeighbors, RadiusSearch::MaxNeighbors());
}

void PCWorker::DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel)
{
    DrawUtils::DrawOnlyNeighbors(pixel, shdDat.ConstPointCloud(), shdDat.ConstNormalCloud(), colorImg
                                 , neighborIndices, numNeighbors, RadiusSearch::MaxNeighbors());
}
