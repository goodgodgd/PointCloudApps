#ifndef OBJECTCLUSTERBASE_H
#define OBJECTCLUSTERBASE_H

#include <utility>
#include <cassert>
#include <QImage>
//#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "Share/project_common.h"
#include "Share/camera_param.h"
#include "Share/shared_data.h"
#include "Share/shared_enums.h"
#include "Share/shared_types.h"
#include "Share/arraydata.h"
#include "ClUtils/cloperators.h"
#include "IO/imageconverter.h"
#include "Test/testinobjcluster.h"
#include "segment.h"
#include "imline.h"

class ObjectClusterBase
{
public:
    enum Enum
    {
        MERGED_PLANE = -1
    };

    ObjectClusterBase();

    void ClusterPlanes(SharedData* shdDat);
    const cl_int* GetObjectMap();
    const vecSegment* GetObjects();
    const Segment* GetObjectByID(const int ID);

protected:
    void InitClustering(SharedData* shdDat);
    virtual void MergePlanes();
    void ExtractValidSegments(const vecSegment& planes, vecSegment& objects);
    bool DoRectsOverlap(const ImRect& leftRect, const ImRect& rightRect);
    ImRect OverlappingRect(const ImRect& leftRect, const ImRect& rightRect);
    bool ArePlanesConnected(const ImRect& ovlRect, const Segment& firstPlane, const Segment& secondPlane, vecPairOfPixels& connPixels);
    bool ArePixelsConnected(const int leftIdx, const cl_float4& leftNormal, const int rightIdx, const cl_float4& rightNormal);
    void AbsorbPlane(Segment& largerPlane, Segment& smallerPlane);

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullityMap;
    vecSegment planes;

    ArrayData<cl_int> objectArray;
    cl_int* objectMap;
    vecSegment objects;
    mapPairOfInts mapIdIndex;
};

#endif // OBJECTCLUSTERBASE_H
