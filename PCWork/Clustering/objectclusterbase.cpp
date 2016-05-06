#include "objectclusterbase.h"

ObjectClusterBase::ObjectClusterBase()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
    , objectMap(nullptr)
{
    objectArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    objectMap = objectArray.GetArrayPtr();
    qDebug() << "ObjectClusterBase";
}

void ObjectClusterBase::ClusterPlanes(SharedData* shdDat)
{
    InitClustering(shdDat);

    MergePlanes();

    ExtractValidSegments(planes, objects);
    qDebug() << "objects" << objects.size();
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
}

void ObjectClusterBase::MergePlanes()
{
}

bool ObjectClusterBase::DoRectsOverlap(const ImRect& leftRect, const ImRect& rightRect)
{
    if(leftRect.xh <= rightRect.xl || rightRect.xh <= leftRect.xl || leftRect.yh <= rightRect.yl || rightRect.yh <= leftRect.yl)
        return false;
    return true;
}

ImRect ObjectClusterBase::OverlappingRect(const ImRect& leftRect, const ImRect& rightRect)
{
    ImRect outRect;
    outRect.xl = smax(leftRect.xl, rightRect.xl);
    outRect.xh = smin(leftRect.xh, rightRect.xh);
    outRect.yl = smax(leftRect.yl, rightRect.yl);
    outRect.yh = smin(leftRect.yh, rightRect.yh);

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

bool ObjectClusterBase::ArePixelsConnected(const int leftIdx, const cl_float4& leftNormal, const int rightIdx, const cl_float4& rightNormal)
{
    const float depth = DEPTH(pointCloud[leftIdx]);
    const float depthDiffLimit = smax(depth*depth*0.005f, 0.003f);

    if(fabsf(clDot(pointCloud[leftIdx] - pointCloud[rightIdx], leftNormal)) < depthDiffLimit)
        return true;
    else if(fabsf(clDot(pointCloud[leftIdx] - pointCloud[rightIdx], rightNormal)) < depthDiffLimit)
        return true;

    return false;
}

void ObjectClusterBase::AbsorbPlane(Segment& largerPlane, Segment& smallerPlane)
{
    for(int y=smallerPlane.rect.yl; y<=smallerPlane.rect.yh; y++)
    {
        for(int x=smallerPlane.rect.xl; x<=smallerPlane.rect.xh; x++)
        {
            if(objectMap[IMGIDX(y,x)]==smallerPlane.id)
                objectMap[IMGIDX(y,x)] = largerPlane.id;
        }
    }
    smallerPlane.id = MERGED_PLANE;
    largerPlane.numpt += smallerPlane.numpt;
    largerPlane.rect.ExpandRange(smallerPlane.rect);
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
        if(plane.id!=MERGED_PLANE)
        {
            mapIdIndex[plane.id] = objects.size();
            objects.push_back(plane);
        }
    }
}

