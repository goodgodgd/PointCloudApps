#include "experimenter.h"

Experimenter::Experimenter()
    : neighborIndices(nullptr)
    , numNeighbors(nullptr)
{
    nullData.Allocate(IMAGE_HEIGHT*IMAGE_WIDTH);
    nullData.SetZero();
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
    CreateNullityMap(shdDat);
    FindPlanes(shdDat);
    SetPlanesNull(shdDat);

    if(bObject)
    {
        CheckObjectValidity(shdDat, DescriptorMaker::DescriptorRadius());
        pclDescs.ComputeObjectDescriptors(shdDat, DescriptorMaker::DescriptorRadius());
        objectRecorder.Record(pclDescs.indicesptr,
                              shdDat->ConstDescriptors(),
                              pclDescs.GetSpinImage(),
                              pclDescs.GetFpfh(),
                              pclDescs.GetShot(),
                              pclDescs.GetTrisi()
                              );
        return;
    }

    const std::vector<TrackPoint>* trackingPoints = pointTracker.Track(shdDat);
    pclDescs.ComputeTrackingDescriptors(shdDat, trackingPoints, DescriptorMaker::DescriptorRadius());
    trackRecorder.Record(trackingPoints,
                           shdDat->ConstDescriptors(),
                           pclDescs.GetSpinImage(),
                           pclDescs.GetFpfh(),
                           pclDescs.GetShot(),
                           pclDescs.GetTrisi()
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

void Experimenter::FindPlanes(SharedData* shdDat)
{
    eltimer.start();
    planeClusterer.Cluster(shdDat);
    shdDat->SetPlaneMap(planeClusterer.GetSegmentMap());
    shdDat->SetPlanes(planeClusterer.GetSegments());

    planeMerger.ClusterPlanes(shdDat);
    shdDat->SetPlaneMap(planeMerger.GetObjectMap());
    shdDat->SetPlanes(planeMerger.GetObjects());

    qDebug() << "planeClusterer took" << eltimer.nsecsElapsed()/1000 << "us";
}

void Experimenter::ComputeDescriptorsCpu(SharedData* shdDat)
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
    const AxesType* prinAxes = descriptorMaker.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetDescAxes(prinAxes);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";
//    qDebug() << "descriptor gpu" << descriptors[IMGIDX(120,160)] << prinAxes[IMGIDX(120,160)];
}

void Experimenter::CheckDataValidity(SharedData* shdDat, const cl_float4* descriptorsGpu, const AxesType* prinAxesGpu)
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
            if(fabsf(prinAxes[i].s[0] - prinAxesGpu[i].s[0]) > 0.001f || fabsf(prinAxes[i].s[4] - prinAxesGpu[i].s[4]) > 0.001f)
                count++;
            if(fabsf(prinAxes[i].s[3]) > 0.001f || fabsf(prinAxes[i].s[7]) > 0.001f)
                count++;
        }
    }
    if(count > 100)
        TryFrameException(QString("too many disagrees %1").arg(count));
}

void Experimenter::CreateNullityMap(SharedData* shdDat)
{
    cl_uchar* nullityMap = nullData.GetArrayPtr();
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const cl_float4* descriptors = shdDat->ConstDescriptors();
    const cl_float8* prinAxes = shdDat->ConstPrinAxes();

    int totalCount=0;
    int descNans=0;
    int axesNans=0;
    int nonZero=0;
    int bigCount1=0;
    int bigCount2=0;
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        nullityMap[i] = NullID::NoneNull;
        if(clIsNull(pointCloud[i]))
            nullityMap[i] = NullID::PointNull;
        else if(clIsNull(normalCloud[i]))
            nullityMap[i] = NullID::NormalNull;
        else if(clIsNull(descriptors[i]))           // must be updated!!
            nullityMap[i] = NullID::DescriptorNull;

        if(nullityMap[i]==NullID::NoneNull)
        {
            ++totalCount;
            if(fabsf(descriptors[i].s[0]) > 115)
                ++bigCount1;
            else if(fabsf(descriptors[i].s[0]) > 105)
                ++bigCount2;

            if(isnanf(descriptors[i].s[0]) || isnanf(descriptors[i].s[1]))
                ++descNans;
            if(isnanf(prinAxes[i].s[0]) || isnanf(prinAxes[i].s[4]))
                ++axesNans;
            if(fabsf(descriptors[i].s[3])>0.0001f)
                ++nonZero;
        }
    }
    qDebug() << "nan" << descNans << axesNans << nonZero << "big" << bigCount1 << bigCount2 << "over" << totalCount;

    shdDat->SetNullityMap(nullityMap);
}

void Experimenter::SetPlanesNull(SharedData* shdDat)
{
    cl_uchar* nullityMap = nullData.GetArrayPtr();
    const vecSegment* planes = shdDat->ConstPlanes();
    const cl_int* planemap = shdDat->ConstPlaneMap();
    const int planeThresh = 5000;
    const QRgb black = qRgb(0,0,0);
    int pxidx;

    for(size_t i=0; i<planes->size(); ++i)
    {
        if(planes->at(i).numpt < planeThresh)
            continue;
        qDebug() << "   large plane" << planes->at(i).id << planes->at(i).numpt << planes->at(i).rect;
        for(int y=planes->at(i).rect.yl; y<=planes->at(i).rect.yh; ++y)
        {
            for(int x=planes->at(i).rect.xl; x<=planes->at(i).rect.xh; ++x)
            {
                pxidx = IMGIDX(y,x);
                if(planemap[pxidx]!=planes->at(i).id)
                    continue;

                if(nullityMap[pxidx]==NullID::NoneNull)
                    nullityMap[pxidx] = NullID::ObjectNull;
                colorImg.setPixel(x, y, black);
            }
        }
    }
    shdDat->SetNullityMap(nullityMap);
    shdDat->SetColorImage(colorImg);
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
    if(validPointCount < 1000)
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
