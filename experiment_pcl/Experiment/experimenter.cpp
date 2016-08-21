#include "experimenter.h"

Experimenter::Experimenter()
    : neighborIndices(nullptr)
    , numNeighbors(nullptr)
{
}

Experimenter::~Experimenter()
{
}

void Experimenter::Work(const QImage& srcColorImg, const QImage& srcDepthImg, const Pose6dof& srcPose, SharedData* shdDat, bool bObject)
{
    shdDat->SetGlobalPose(srcPose);
    shdDat->SetColorImage(srcColorImg);
    colorImg = srcColorImg;
    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(srcDepthImg);
    shdDat->SetPointCloud(pointCloud);

    SearchNeighborsAndCreateNormal(shdDat);
//    ComputeDescriptorsCpu(shdDat);
    ComputeDescriptorsGpu(shdDat);
    const cl_uchar* nullityMap = CreateNullityMap(shdDat);
    shdDat->SetNullityMap(nullityMap);

    qDebug() << "pose" << srcPose;
//    return;

    if(bObject)
    {
        CheckObjectValidity(shdDat, DescriptorMaker::DescriptorRadius());
        pclDescs.ComputeObjectDescriptors(shdDat, DescriptorMaker::DescriptorRadius());
        objectRecorder.Record(pclDescs.indicesptr,
                              shdDat->ConstDescriptors(),
                              pclDescs.GetSpinImage(),
                              pclDescs.GetFpfh(),
                              pclDescs.GetShot()
                              );
        return;
    }

    const std::vector<TrackPoint>* trackingPoints = pointTracker.Track(shdDat);
    pclDescs.ComputeTrackingDescriptors(shdDat, trackingPoints, DescriptorMaker::DescriptorRadius());
    trackRecorder.Record(trackingPoints,
                           shdDat->ConstDescriptors(),
                           pclDescs.GetSpinImage(),
                           pclDescs.GetFpfh(),
                           pclDescs.GetShot()
                           );
}

void Experimenter::SearchNeighborsAndCreateNormal(SharedData* shdDat)
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

void Experimenter::ComputeDescriptorsCpu(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const DescType* descriptors = nullptr;
    const AxesType* descAxes = nullptr;

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, DescriptorMakerByCpu::DescriptorRadius()
                                       , DescriptorMakerByCpu::DescriptorNeighbors(), CameraParam::flh());
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborsForDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    descriptorMakerCpu.ComputeDescriptors(pointCloud, normalCloud, neighborIndices, numNeighbors, RadiusSearch::MaxNeighbors());
    descriptors = descriptorMakerCpu.GetDescriptors();
    descAxes = descriptorMakerCpu.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetDescAxes(descAxes);
    qDebug() << "ComputeDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "descriptor cpu" << descriptors[IMGIDX(100,100)] << descAxes[IMGIDX(100,100)];

    // ========== check validity ==========
    ComputeDescriptorsGpu(shdDat);

    CheckDataValidity(shdDat, descriptors, descAxes);
}

void Experimenter::ComputeDescriptorsGpu(SharedData* shdDat)
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
    const AxesType* descAxes = descriptorMaker.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetDescAxes(descAxes);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "descriptor gpu" << descriptors[IMGIDX(120,160)] << descAxes[IMGIDX(120,160)];
}

void Experimenter::CheckDataValidity(SharedData* shdDat, const cl_float4* descriptorsGpu, const AxesType* descAxesGpu)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const DescType* descriptors = shdDat->ConstDescriptors();
    const AxesType* descAxes = shdDat->ConstDescAxes();
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

        clSplit(descAxes[i], majorAxis, minorAxis);
        assert(clIsNormalized(majorAxis) || clIsNull(majorAxis));
        assert(clIsNormalized(minorAxis) || clIsNull(minorAxis));

        if(descriptorsGpu!=nullptr)
            if(clLength(descriptors[i] - descriptorsGpu[i]) > 0.001f)
                count++;
        if(descAxesGpu!=nullptr)
        {
            if(fabsf(descAxes[i].s[0] - descAxesGpu[i].s[0]) > 0.001f || fabsf(descAxes[i].s[4] - descAxesGpu[i].s[4]) > 0.001f)
                count++;
            if(fabsf(descAxes[i].s[3]) > 0.001f || fabsf(descAxes[i].s[7]) > 0.001f)
                count++;
        }
    }
    if(count > 100)
        TryFrameException(QString("too many disagrees %1").arg(count));
}

cl_uchar* Experimenter::CreateNullityMap(SharedData* shdDat)
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

void Experimenter::CheckObjectValidity(SharedData* shdDat, const float minSize)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();
    Range3D<float> range(-10.f, 10.f, -10.f, 10.f, -10.f, 10.f);

    int validPointCount=0;
    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        if(nullityMap[i]!=NullID::NoneNull)
            continue;
        range.ExpandRange(pointCloud[i]);
        validPointCount++;
    }
    if(validPointCount < 500)
        throw TryFrameException(QString("invalid object points: %1").arg(validPointCount));

    int validDimCount=0;
    if(range.Depth() > minSize)
        validDimCount++;
    if(range.Width() > minSize)
        validDimCount++;
    if(range.Height() > minSize)
        validDimCount++;

    if(validDimCount<2)
        throw TryFrameException(QString("invalid object size: %1, %2, %3").arg(range.Depth()).arg(range.Width()).arg(range.Height()));
}
