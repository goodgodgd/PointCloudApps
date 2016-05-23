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
#include "segment.h"

class ClustererByDbRect
{
public:
    ClustererByDbRect();
    void FindDbObjects(SharedData* shdDat, const vecAnnot& annots);
    const cl_int* GetObjectMap();
    const vecSegment* GetObjects();
};

#endif // CLUSTERERBYDBRECT_H
