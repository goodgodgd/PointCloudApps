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

    if(bObject)
        g_descriptorRadius = CheckObjectSize(shdDat);
    CreateNormalAndNullity(shdDat);

    ComputeCWGDescriptor(shdDat);

    if(bObject)
    {
        pclDescs.ComputeObjectDescriptors(shdDat);
        objectRecorder.Record(pclDescs.indicesptr,
                              shdDat->ConstDescriptors(),
                              pclDescs.GetSpinImage(),
                              pclDescs.GetFpfh(),
                              pclDescs.GetShot()
                              );
        return;
    }

    const std::vector<TrackPoint>* trackingPoints = pointTracker.Track(shdDat);

    pclDescs.ComputeTrackingDescriptors(shdDat, trackingPoints);

    trackRecorder.Record(trackingPoints,
                           shdDat->ConstDescriptors(),
                           pclDescs.GetSpinImage(),
                           pclDescs.GetFpfh(),
                           pclDescs.GetShot()
                           );
}

float Experimenter::CheckObjectSize(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    float ymin=10.f, ymax=-10.f, zmin=10.f, zmax=-10.f;
    float descRadius = DESCRIPTOR_RADIUS;

    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        if(clIsNull(pointCloud[i]))
            continue;

        if(pointCloud[i].y < ymin)
            ymin = pointCloud[i].y;
        if(pointCloud[i].y > ymax)
            ymax = pointCloud[i].y;
        if(pointCloud[i].z < zmin)
            zmin = pointCloud[i].z;
        if(pointCloud[i].z > zmax)
            zmax = pointCloud[i].z;
    }

    if(ymax - ymin < descRadius*1.5f || zmax - zmin < descRadius*1.5f)
    {
        descRadius = smin(ymax - ymin, zmax - zmin)*0.3f;
        qDebug() << "change descriptor radius" << descRadius << "size" << ymax - ymin << zmax - zmin;
    }
    return descRadius;
}

void Experimenter::CreateNormalAndNullity(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, g_descriptorRadius, CameraParam::flh(), NEIGHBORS_PER_POINT);
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborIndices took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices
                              , neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    const cl_float4* normalCloud = normalMaker.GetNormalCloud();
    shdDat->SetNormalCloud(normalCloud);
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";

    const cl_uchar* nullityMap = CreateNullityMap(shdDat);
    shdDat->SetNullityMap(nullityMap);
    CheckDataValidity(pointCloud, normalCloud);
}

void Experimenter::CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud)
{
    // 1. w channel of point cloud, normal cloud and descriptors must be "0"
    // 2. length of normal is either 0 or 1
    // 3. w channel of descriptors is "1" if valid, otherwise "0"
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        assert(pointCloud[i].w==0.f);
        assert(normalCloud[i].w==0.f);
        assert(fabsf(clLength(normalCloud[i])-1.f) < 0.0001f || clLength(normalCloud[i]) < 0.0001f);
    }
}

cl_uchar* Experimenter::CreateNullityMap(SharedData* shdDat)
{
    static ArrayData<cl_uchar> nullData(IMAGE_HEIGHT*IMAGE_WIDTH);
    cl_uchar* nullityMap = nullData.GetArrayPtr();

    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    int nnCount=0;

    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        nullityMap[i] = NullID::NoneNull;
        if(clIsNull(pointCloud[i]))
            nullityMap[i] = NullID::PointNull;
        else if(clIsNull(normalCloud[i]))
            nullityMap[i] = NullID::NormalNull;
        else
            nnCount++;
    }
    qDebug() << "valid points are" << nnCount;
    if(nnCount < 700)
        throw TryFrameException("too few valid points");
    return nullityMap;
}

void Experimenter::ComputeCWGDescriptor(SharedData* shdDat)
{
//    eltimer.start();
//    cwgMaker_cpu.ComputeDescriptors(shdDat->ConstPointCloud(), shdDat->ConstNormalCloud(), neighborIndices, numNeighbors, NEIGHBORS_PER_POINT);
//    shdDat->SetDescriptors(cwgMaker_cpu.GetDescriptor());
//    qDebug() << "CWG CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    cwgMaker_gpu.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    shdDat->SetDescriptors(cwgMaker_gpu.GetDescriptor());
    qDebug() << "CWG GPU took" << eltimer.nsecsElapsed()/1000 << "us";
}
