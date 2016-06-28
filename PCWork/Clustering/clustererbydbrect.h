#ifndef CLUSTERERBYDBRECT_H
#define CLUSTERERBYDBRECT_H

#include <cassert>
#include "Share/project_common.h"
#include "Share/camera_param.h"
#include "Share/shared_data.h"
#include "Share/shared_enums.h"
#include "Share/shared_types.h"
#include "Share/arraydata.h"
#include "Share/annotation.h"
#include "ClUtils/cloperators.h"
#include "objectclusterbase.h"
#include "segment.h"

class ClustererByDbRect
{
public:
    ObjectClusterBase objectclusterbase;
    ClustererByDbRect();
    void FindDbObjects(SharedData* shdDat, const vecAnnot& annots);
    const cl_int* GetObjectMap();
    const vecSegment* GetObjects();
    void SetObjects(const vecSegment* srcptr) { objects = srcptr; }
    bool DoRectsEqual(const ImRect& firstRect, const ImRect& secondRect);
    const vecSegment* objects;
    ArrayData<cl_int> objectArray;
    cl_int* objectMap;
};

#endif // CLUSTERERBYDBRECT_H
