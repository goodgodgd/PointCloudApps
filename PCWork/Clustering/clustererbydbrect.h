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
    bool DoRectsInclude(const ImRect& outerRect, const ImRect& innerRect);
    bool DoRectsOverlap(const ImRect& firstRect, const ImRect& secondRect);
    ImRect OverlappingRect(const ImRect& firstRect, const ImRect& secondRect);
    float GetRectsIOU(const ImRect& intersectRect, const ImRect& innerRect);
    const vecSegment* objects;
    ArrayData<cl_int> objectArray;
    cl_int* objectMap;
};

struct fixList
{
    fixList(int id_, int xl_, int xh_, int yl_, int yh_)
    {
        id = id_;
        rect = ImRect(xl_,xh_,yl_,yh_);
    }
    int id;
    ImRect rect;
};
typedef std::vector<fixList> vecfixList;

#endif // CLUSTERERBYDBRECT_H
