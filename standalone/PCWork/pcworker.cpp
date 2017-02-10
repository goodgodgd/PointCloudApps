#include "pcworker.h"

PCWorker::PCWorker()
    : neighborIndices(nullptr)
    , numNeighbors(nullptr)
{
    nullData.Allocate(IMAGE_HEIGHT*IMAGE_WIDTH);

    timingFile.setFileName("timing.txt");
    if(timingFile.open(QIODevice::WriteOnly|QIODevice::Text)==false)
        throw TryFrameException("cannot open depth file");
}

PCWorker::~PCWorker()
{
}

void PCWorker::Work(const QImage& srcColorImg, const QImage& srcDepthImg, const Pose6dof& framePose, SharedData* shdDat/*, vector<AnnotRect>& annotRects*/)
{
    shdDat->SetColorImage(srcColorImg);
    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(srcDepthImg);
    shdDat->SetPointCloud(pointCloud);

    SearchNeighborsAndCreateNormal(shdDat);
    SetBasicNullity(shdDat);

//    ComputeDescriptorsCpu(shdDat);
    ComputeDescriptorsGpu(shdDat);
    SetDescriptorNullity(shdDat);

    return;

    ClusterPointsOfObjects(shdDat);
}

void PCWorker::SearchNeighborsAndCreateNormal(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    QTextStream timeWriter(&timingFile);
    int searchtime, normaltime;

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, NormalMaker::NormalRadius(), NormalMaker::NormalNeighbors(), CameraParam::flh());
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    searchtime = eltimer.nsecsElapsed()/1000;
    qDebug() << "SearchNeighborForNormal took" << searchtime << "us";

    eltimer.start();
    normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, RadiusSearch::MaxNeighbors());
    const cl_float4* normalCloud = normalMaker.GetNormalCloud();
    shdDat->SetNormalCloud(normalCloud);
    normaltime = eltimer.nsecsElapsed()/1000;
    qDebug() << "ComputeNormal took" << normaltime << "us";

    timeWriter << normaltime << " " << searchtime << " ";
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
    shdDat->SetPrinAxes(prinAxes);
    qDebug() << "ComputeDescriptorCpu took" << eltimer.nsecsElapsed()/1000 << "us";

//    return;
    // ========== check validity ==========
    ComputeDescriptorsGpu(shdDat);

    CheckDataValidity(shdDat, descriptors, prinAxes);
}

void PCWorker::ComputeDescriptorsGpu(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    QTextStream timeWriter(&timingFile);
    int desctime;

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
    shdDat->SetPrinAxes(prinAxes);
    desctime = eltimer.nsecsElapsed()/1000;
    qDebug() << "ComputeDescriptorCpu took" << desctime << "us";

    const cl_uchar* nullity = shdDat->ConstNullityMap();
    int nullcnt=0;
    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
        if(nullity[i]==NullID::NoneNull)
            nullcnt++;

    timeWriter << desctime << " " << nullcnt << "\n";
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
    int validCnt=0, descCnt=0, gradCnt=0, zeroCnt=0;
    int axesCnt[] = {0,0,0,0};
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        assert(pointCloud[i].w==0.f);
        assert(normalCloud[i].w==0.f);
        assert(clIsNormalized(normalCloud[i]) || clIsNull(normalCloud[i]));

        clSplit(prinAxes[i], majorAxis, minorAxis);
        assert(clIsNormalized(majorAxis) || clIsNull(majorAxis));
        assert(clIsNormalized(minorAxis) || clIsNull(minorAxis));

        if(clIsNull(descriptors[i]))
            continue;
        validCnt++;

        if(descriptorsGpu==nullptr)
            continue;

        if(fabsf(descriptors[i].x - descriptorsGpu[i].x) > 0.001f || fabsf(descriptors[i].y - descriptorsGpu[i].y) > 0.001f)
        {
            descCnt++;
            if(i%1000==100)
                qDebug() << "descCnt" << i << descCnt << descriptors[i] << descriptorsGpu[i];
        }
        if(fabsf(descriptors[i].z - descriptorsGpu[i].z) > 0.001f || fabsf(descriptors[i].w - descriptorsGpu[i].w) > 0.001f)
        {
            gradCnt++;
            if(i%200==100)
                qDebug() << "gradCnt" << i << gradCnt << descriptors[i] << descriptorsGpu[i];
        }

        if(prinAxesGpu==nullptr)
            continue;
        if(fabsf(descriptors[i].z) < 0.001f || fabsf(descriptors[i].w) < 0.001f)
            continue;

        if(fabsf(prinAxes[i].s[0] + prinAxesGpu[i].s[0]) < 0.001f)
        {
            axesCnt[0]++;
            if(i%1000==100)
                qDebug() << "axesCnt[0]" << i << axesCnt[0] << descriptors[i] << descriptorsGpu[i] << "\n   " << prinAxes[i] << prinAxesGpu[i];
        }
        else if(fabsf(prinAxes[i].s[4] + prinAxesGpu[i].s[4]) < 0.001f)
        {
            axesCnt[1]++;
            if(i%1000==100)
                qDebug() << "axesCnt[1]" << i << axesCnt[1] << prinAxes[i] << prinAxesGpu[i];
        }
        else if(fabsf(prinAxes[i].s[0] - prinAxesGpu[i].s[0]) > 0.001f)
        {
            axesCnt[2]++;
            if(i%1000==100)
                qDebug() << "axesCnt[2]" << i << axesCnt[2] << prinAxes[i] << prinAxesGpu[i];
        }
        else if(fabsf(prinAxes[i].s[4] - prinAxesGpu[i].s[4]) > 0.001f)
        {
            axesCnt[3]++;
            if(i%1000==100)
                qDebug() << "axesCnt[3]" << i << axesCnt[3] << prinAxes[i] << prinAxesGpu[i];
        }

        if(fabsf(prinAxes[i].s[3]) > 0.001f)
            zeroCnt++;
        else if(fabsf(prinAxes[i].s[7]) > 0.001f)
            zeroCnt++;

    }
    qDebug() << "descriptor disagree" << descCnt << gradCnt
             << "axesCnt" << axesCnt[0] << axesCnt[1] << axesCnt[2] << axesCnt[3] << zeroCnt << "out of" << validCnt;
}

void PCWorker::SetBasicNullity(SharedData* shdDat)
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

        if(numNeighbors[i] < 50/2)
            neibLack++;


        if(clIsNull(pointCloud[i]))
            pointNull++;
        if(clIsNull(normalCloud[i]))
            normalNull++;
    }
    qDebug() << "null point normal" << pointNull << normalNull << neibLack;
    shdDat->SetNullityMap(nullityMap);
}

void PCWorker::SetDescriptorNullity(SharedData* shdDat)
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
    qDebug() << "nan descriptors" << nanCount << nullCount;
    shdDat->SetNullityMap(nullityMap);
}

void PCWorker::MarkNeighborsOnImage(QImage& srcimg, QPoint pixel)
{
    DrawUtils::MarkNeighborsOnImage(srcimg, pixel, neighborIndices, numNeighbors, RadiusSearch::MaxNeighbors());
}

void PCWorker::DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel)
{
    DrawUtils::DrawOnlyNeighbors(pixel, shdDat.ConstPointCloud(), shdDat.ConstNormalCloud(), shdDat.ConstColorImage()
                                 , neighborIndices, numNeighbors, RadiusSearch::MaxNeighbors());
}
