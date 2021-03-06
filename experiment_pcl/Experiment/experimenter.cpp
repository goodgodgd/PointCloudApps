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

void Experimenter::Work(SharedData* shdDat, const ExpmUiOptions options)
{
    SearchNeighborsAndCreateNormal(shdDat);
    SetBasicNullity(shdDat);

//    ComputeDescriptorsCpu(shdDat);
    ComputeDescriptorsGpu(shdDat);
    SetDescriptorNullity(shdDat);

    if(options.target == ExpmTarget::OBJECT)
    {
        CheckObjectValidity(shdDat, DESC_RADIUS);
        pclDescs.ComputeObjectDescriptors(shdDat, DESC_RADIUS);
        if(options.writeDesc)
            objectRecorder.Record(pclDescs.indicesptr,
                                  shdDat->ConstDescriptors(),
                                  pclDescs.GetSpinImage(),
                                  pclDescs.GetFpfh(),
                                  pclDescs.GetShot(),
                                  pclDescs.GetTrisi()
                                  );
        return;
    }

    FindPlanes(shdDat);
    SetPlanesNull(shdDat);

    const std::vector<TrackPoint>* trackingPoints;
    if(options.target == ExpmTarget::SCENE)
        trackingPoints = pointTracker.SamplePoints(shdDat);
    else if(options.target == ExpmTarget::SAMPLE)
        trackingPoints = GetTrackPoint(shdDat);
    else
        return;
    qDebug() << "trackpoints" << trackingPoints->size();
    if(trackingPoints->empty())
        return;

    pclDescs.ComputeTrackingDescriptors(shdDat, trackingPoints, DESC_RADIUS);
    if(options.writeDesc)
    {
        bool bNewFile = true;
        if(options.target == ExpmTarget::SAMPLE)
            bNewFile = false;
        trackRecorder.Record(shdDat, trackingPoints,
                             pclDescs.GetSpinImage(),
                             pclDescs.GetFpfh(),
                             pclDescs.GetShot(),
                             pclDescs.GetTrisi(),
                             bNewFile
                             );
    }
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
    neibSearcher.SearchNeighborIndices(pointCloud, DESC_RADIUS, NUM_NEIGHBORS, CameraParam::flh());
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborsForDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    descriptorMakerCpu.ComputeDescriptors(pointCloud, normalCloud, neighborIndices, numNeighbors, RadiusSearch::MaxNeighbors());
    descriptors = descriptorMakerCpu.GetDescriptors();
    prinAxes = descriptorMakerCpu.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetPrinAxes(prinAxes);
    qDebug() << "ComputeDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";
    qDebug() << "descriptor cpu" << descriptors[IMGIDX(100,100)] << prinAxes[IMGIDX(100,100)];

    return;
    // ========== check validity ==========
    ComputeDescriptorsGpu(shdDat);

    CheckDataValidity(shdDat, descriptors, prinAxes);
}

void Experimenter::ComputeDescriptorsGpu(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, DESC_RADIUS, NUM_NEIGHBORS, CameraParam::flh());
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();

    descriptorMaker.ComputeDescriptors(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, RadiusSearch::MaxNeighbors());
    const DescType* descriptors = descriptorMaker.GetDescriptor();
    const AxesType* prinAxes = descriptorMaker.GetDescAxes();
    shdDat->SetDescriptors(descriptors);
    shdDat->SetPrinAxes(prinAxes);
    qDebug() << "ComputeDescriptor took" << eltimer.nsecsElapsed()/1000 << "us";
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
    int curvCount=0, axesCount=0;
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
                curvCount++;
        if(prinAxesGpu!=nullptr)
            if(fabsf(prinAxes[i].s[0] - prinAxesGpu[i].s[0]) > 0.001f || fabsf(prinAxes[i].s[4] - prinAxesGpu[i].s[4]) > 0.001f)
                axesCount++;
    }
    qDebug() << "cpu gpu disagree" << curvCount << axesCount;
    if(curvCount+axesCount > 100)
        TryFrameException(QString("too many disagrees %1 %2").arg(curvCount).arg(axesCount));
}

void Experimenter::SetPlanesNull(SharedData* shdDat)
{
    cl_uchar* nullityMap = nullData.GetArrayPtr();
    const vecSegment* planes = shdDat->ConstPlanes();
    const cl_int* planemap = shdDat->ConstPlaneMap();
#ifdef SCALE_VAR
    const int planeThresh = smax(3000*4/SCALE_VAR/SCALE_VAR, 3000);
#else
    const int planeThresh = 3000;
#endif
    const QRgb black = qRgb(0,0,0);
    int pxidx;
    QImage colorImg = shdDat->ConstColorImage();

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

void Experimenter::SetBasicNullity(SharedData* shdDat)
{
    cl_uchar* nullityMap = nullData.GetArrayPtr();

    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    int pointNull=0, normalNull=0, neibLack=0;

    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        nullityMap[i] = NullID::NoneNull;
        if(clIsNull(pointCloud[i]))
            nullityMap[i] = NullID::PointNull;
        else if(clIsNull(normalCloud[i]))
            nullityMap[i] = NullID::NormalNull;

        if(numNeighbors[i] < NUM_NEIGHBORS/2)
            neibLack++;


        if(clIsNull(pointCloud[i]))
            pointNull++;
        if(clIsNull(normalCloud[i]))
            normalNull++;
    }
    qDebug() << "null point normal" << pointNull << normalNull << neibLack;
    shdDat->SetNullityMap(nullityMap);
}

void Experimenter::SetDescriptorNullity(SharedData* shdDat)
{
    cl_uchar* nullityMap = nullData.GetArrayPtr();
    const cl_float4* descriptors = shdDat->ConstDescriptors();
    int nanCount=0, nullCount=0, valiCount=0;

    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        if(nullityMap[i] < NullID::NormalNull && clIsNull(descriptors[i]))
            nullityMap[i] = NullID::DescriptorNull;

        if(clIsNan(descriptors[i]))
            ++nanCount;
        else if(clIsNull(descriptors[i]))
            ++nullCount;
        else
            ++valiCount;
    }
//    qDebug() << "nan descriptors" << nanCount << nullCount;
    shdDat->SetNullityMap(nullityMap);
}

const std::vector<TrackPoint>* Experimenter::GetTrackPoint(SharedData* shdDat)
{
    static std::vector<TrackPoint> tracks;
    tracks.clear();
    const std::vector<int> imgpt = shdDat->ConstTargetPixel();
    if(imgpt.size() < 2)
        throw TryFrameException(QString("invalid target pixel"));
    cl_uint2 pixel = (cl_uint2){(cl_uint)imgpt[0], (cl_uint)imgpt[1]};

    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const cl_float8* praxesCloud = shdDat->ConstPrinAxes();
    const int pxidx = PIXIDX(pixel);
    if(clIsNull(pointCloud[pxidx]) ||
        clIsNull(normalCloud[pxidx]) ||
        clIsNull(praxesCloud[pxidx]))
    {
        qDebug() << "GetTrackPoint, invalid track:" << pointCloud[pxidx] << normalCloud[pxidx] << praxesCloud[pxidx];
        return &tracks;
    }

    TrackPoint track;
    track.ID = 1;
    track.beginIndex = g_frameIdx;
    track.frameIndex = g_frameIdx;
    track.gpoint = pointCloud[pxidx];
    track.gnormal = normalCloud[pxidx];
    track.pixel = pixel;
    track.tcount = 1;

    tracks.push_back(track);
    return &tracks;
}


