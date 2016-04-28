#include "objectclusterer.h"

ObjectClusterer::ObjectClusterer()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
    , objectMap(nullptr)
{
    objectArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    objectMap = objectArray.GetArrayPtr();
}

void ObjectClusterer::ClusterCloudIntoObjects(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();
    memcpy(objectMap, shdDat->ConstPlaneMap(), sizeof(cl_int)*IMAGE_WIDTH*IMAGE_HEIGHT);
    objects = *(shdDat->ConstPlanes());

    ClusterPlanes(objectMap, objects);
}

void ObjectClusterer::ClusterPlanes(cl_int* objectMap, vecSegment& objects)
{
    typedef vecSegment::const_iterator    vsegIter;

    for(vsegIter ref=objects.begin(); ref+1!=objects.end(); ref++)
    {
        for(vsegIter cmp=ref+1; cmp!=objects.end(); cmp++)
        {

        }
    }
}

void ObjectClusterer::MergePlanesConvexToEachOther()
{
}

const cl_int* ObjectClusterer::GetObjectMap()
{
    return objectMap;
}

const vecSegment* ObjectClusterer::GetObjects()
{
    return &objects;
}
