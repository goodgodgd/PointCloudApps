#ifndef OBJECTCLUSTERER_H
#define OBJECTCLUSTERER_H

#include <cassert>
#include "Share/project_common.h"
#include "Share/shareddata.h"
#include "Share/arraydata.h"
#include "ClUtils/cloperators.h"
#include "segment.h"

class ObjectClusterer
{
public:
    ObjectClusterer();

    void ClusterCloudIntoObjects(SharedData* shdDat);
    const cl_int* GetObjectMap();
    const vecSegment* GetObjects();

private:
    void ClusterPlanes(cl_int* objectMap, vecSegment& objects);
    void MergePlanesConvexToEachOther();

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;

    ArrayData<cl_int> objectArray;
    cl_int* objectMap;
    vecSegment objects;

};

#endif // OBJECTCLUSTERER_H
