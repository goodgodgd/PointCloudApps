#include "objectclusterbase.h"

ObjectClusterBase::ObjectClusterBase()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
    , objectMap(nullptr)
{
    objectArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    objectMap = objectArray.GetArrayPtr();

    // for debug
    srcPlaneArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    srcPlaneMap = srcPlaneArray.GetArrayPtr();
}

void ObjectClusterBase::ClusterPlanes(SharedData* shdDat)
{
    InitClustering(shdDat);

    MergePlanes();

    ExtractValidSegments(planes, objects);
    qDebug() << "   # objects" << objects.size();
}

void ObjectClusterBase::InitClustering(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();
    memcpy(objectMap, shdDat->ConstPlaneMap(), sizeof(cl_int)*IMAGE_WIDTH*IMAGE_HEIGHT);
    planes = *(shdDat->ConstPlanes());
    std::sort(planes.begin(), planes.end(), [](const Segment& a, const Segment& b)
                                              { return a.numpt > b.numpt; });
    objects.clear();

    // for debug
    memcpy(srcPlaneMap, shdDat->ConstPlaneMap(), sizeof(cl_int)*IMAGE_WIDTH*IMAGE_HEIGHT);
    srcPlanes = *(shdDat->ConstPlanes());
}

bool ObjectClusterBase::DoRectsOverlap(const ImRect& firstRect, const ImRect& secondRect)
{
    if(firstRect.xh <= secondRect.xl || secondRect.xh <= firstRect.xl || firstRect.yh <= secondRect.yl || secondRect.yh <= firstRect.yl)
        return false;
    return true;
}

ImRect ObjectClusterBase::OverlappingRect(const ImRect& firstRect, const ImRect& secondRect)
{
    ImRect outRect;
    outRect.xl = smax(firstRect.xl, secondRect.xl);
    outRect.xh = smin(firstRect.xh, secondRect.xh);
    outRect.yl = smax(firstRect.yl, secondRect.yl);
    outRect.yh = smin(firstRect.yh, secondRect.yh);

    outRect.xl = smax(outRect.xl-1, 0);
    outRect.xh = smin(outRect.xh+1, IMAGE_WIDTH-1);
    outRect.yl = smax(outRect.yl-1, 0);
    outRect.yh = smin(outRect.yh+1, IMAGE_HEIGHT-1);
    return outRect;
}

bool ObjectClusterBase::ArePlanesConnected(const ImRect& ovlRect, const Segment& firstPlane, const Segment& secondPlane, vecPairOfPixels& connPixels)
{
    int pixidx, rigidx, lowidx;
    int interfaceCount=0;
    connPixels.clear();

    for(int y=ovlRect.yl; y<ovlRect.yh; y++)
    {
        for(int x=ovlRect.xl; x<ovlRect.xh; x++)
        {
            pixidx = IMGIDX(y,x);
            rigidx = IMGIDX(y,x+1);
            lowidx = IMGIDX(y+1,x);

            if(objectMap[pixidx]==firstPlane.id && objectMap[rigidx]==secondPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(pixidx, firstPlane.normal, rigidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x,y},(cl_int2){x+1,y});
            }
            if(objectMap[pixidx]==secondPlane.id && objectMap[rigidx]==firstPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(rigidx, firstPlane.normal, pixidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x+1,y}, (cl_int2){x,y});
            }

            if(objectMap[pixidx]==firstPlane.id && objectMap[lowidx]==secondPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(pixidx, firstPlane.normal, lowidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x,y},(cl_int2){x,y+1});
            }
            if(objectMap[pixidx]==secondPlane.id && objectMap[lowidx]==firstPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(lowidx, firstPlane.normal, pixidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x,y+1}, (cl_int2){x,y});
            }
        }
    }
    return (connPixels.size() > interfaceCount/2 && connPixels.size() > 3);
}

bool ObjectClusterBase::ArePixelsConnected(const int firstIdx, const cl_float4& firstNormal, const int secondIdx, const cl_float4& secondNormal)
{
    const float depth = DEPTH(pointCloud[firstIdx]);
    const float depthDiffLimit = smax(depth*depth*0.005f, 0.003f);

    if(fabsf(clDot(pointCloud[firstIdx] - pointCloud[secondIdx], firstNormal)) < depthDiffLimit)
        return true;
    else if(fabsf(clDot(pointCloud[firstIdx] - pointCloud[secondIdx], secondNormal)) < depthDiffLimit)
        return true;

    return false;
}

void ObjectClusterBase::AbsorbPlane(Segment& basePlane, Segment& mergedPlane)
{
    if(basePlane.id == mergedPlane.id)
        return;
    for(int y=mergedPlane.rect.yl; y<=mergedPlane.rect.yh; y++)
    {
        for(int x=mergedPlane.rect.xl; x<=mergedPlane.rect.xh; x++)
        {
            if(objectMap[IMGIDX(y,x)]==mergedPlane.id)
                objectMap[IMGIDX(y,x)] = basePlane.id;
        }
    }
    mergedPlane.id = Segment::SEG_INVALID;
    basePlane.numpt += mergedPlane.numpt;
    basePlane.rect.ExpandRange(mergedPlane.rect);
}

const cl_int* ObjectClusterBase::GetObjectMap()
{
    return objectMap;
}

const vecSegment* ObjectClusterBase::GetObjects()
{
    return &objects;
}

const Segment* ObjectClusterBase::GetObjectByID(const int ID)
{
    if(mapIdIndex.find(ID)!=mapIdIndex.end())
        return &objects[mapIdIndex[ID]];
    else
        return nullptr;
}

void ObjectClusterBase::ExtractValidSegments(const vecSegment& planes, vecSegment& objects)
{
    for(auto& plane : planes)
    {
        if(plane.id!=Segment::SEG_INVALID)
        {
            mapIdIndex[plane.id] = objects.size();
            objects.push_back(plane);
        }
    }
}

Segment* ObjectClusterBase::GetPlaneByID(const int ID)
{
    for(Segment& plane: srcPlanes)
    {
        if(plane.id==ID)
            return &plane;
    }
    return nullptr;
}

float ObjectClusterBase::HeightFromPlane(const Segment& inputPlane, const Segment& basePlane, const bool bAbs)
{
    auto updateDist = GetDistUpdater(bAbs);
    const float baseDist = fabsf(clDot(basePlane.center, basePlane.normal));
    float extremeDist = 0;
    int yitv = smax((inputPlane.rect.yh - inputPlane.rect.yl)/10, 1);
    int xitv = smax((inputPlane.rect.xh - inputPlane.rect.xl)/10, 1);
    int pxidx;

    for(int y=inputPlane.rect.yl; y<=inputPlane.rect.yh; y+=yitv)
    {
        for(int x=inputPlane.rect.xl; x<=inputPlane.rect.xh; x+=xitv)
        {
            pxidx = IMGIDX(y,x);
            if(nullityMap[pxidx] < NullID::PointNull && objectMap[pxidx]==inputPlane.id)
                extremeDist = updateDist(extremeDist, baseDist - fabsf(clDot(pointCloud[pxidx], basePlane.normal)));
        }
    }
    return extremeDist;
}

std::function<float(float,float)> ObjectClusterBase::GetDistUpdater(bool bAbs)
{
    static auto getMaxHeight = [](float a, float b) { return smax(a, b); };
    static auto getMaxDist = [](float a, float b) { return smax(fabsf(a), fabsf(b)); };
    if(bAbs)
        return getMaxDist;
    else
        return getMaxHeight;
}
