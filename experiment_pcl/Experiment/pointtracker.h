#ifndef POINTTRACKER_H
#define POINTTRACKER_H

#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/pose6dof.h"
#include "Share/arraydata.h"
#include "IO/drawutils.h"
#include "IO/imageconverter.h"
#include "IO/drawutils.h"
#include "ClUtils/cloperators.h"
#include "ShareExpm/expm_common.h"
#include "ShareExpm/expm_structs.h"

class PointTracker
{
public:
    PointTracker();
    const std::vector<TrackPoint>* Track(SharedData* shdDat);

private:
    std::vector<TrackPoint> TrackPoints(const std::vector<TrackPoint>& srcPoints);
    cl_uint2 ProjectOntoImage(const cl_float4& gpoint, const Pose6dof& pose);
    cl_uint2 SearchCorrespondingPixel(const TrackPoint& srcPoints);
    TrackPoint UpdateTrackPoint(const TrackPoint& srcPoint, const cl_uint2& selPixel);
    std::vector<TrackPoint> SampleNewPoints();
    bool FindAdjacentPoint(const cl_uint2& pixel, const uchar* occpMap, const cl_float4* pointCloud);
    void DrawTrackingPoints(const std::vector<TrackPoint>& trackingPoints, const int NumExisting);

    std::vector<TrackPoint> trackingPoints;
    ArrayData<uchar> occupArray;
    int IDcount;
    int NumExisting;

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullity;
    Pose6dof curPose;
};

#endif // POINTTRACKER_H
