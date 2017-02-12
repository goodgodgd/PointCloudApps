#ifndef POINTTRACKER_H
#define POINTTRACKER_H

#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/pose6dof.h"
#include "Share/arraydata.h"
#include "Share/camera_param.h"
#include "IO/drawutils.h"
#include "IO/imageconverter.h"
#include "IO/drawutils.h"
#include "ClUtils/cloperators.h"
#include "ShareExpm/expm_common.h"
#include "ShareExpm/expm_structs.h"

class PointTracker
{
//#define SAMPLE_RANGE    1.2f
//#define TRACK_RANGE     1.5f
public:
    PointTracker();
    const std::vector<TrackPoint>* Track(SharedData* shdDat);
    const std::vector<TrackPoint>* SamplePoints(SharedData* shdDat);

private:
    int TrackPoints(std::vector<TrackPoint>& srcPoints);
    cl_uint2 ProjectOntoImage(const cl_float4& gpoint, const Pose6dof& pose);
    cl_uint2 SearchCorrespondingPixel(const TrackPoint& srcPoints);
    void UpdateTrackPoint(TrackPoint& srcPoint, const cl_uint2& selPixel);
    void AppendNewTracks(std::vector<TrackPoint>& trackPoints);
    bool FindAdjacentPoint(const cl_uint2& pixel, const uchar* occpMap, const cl_float4* pointCloud, const cl_float4* normalCloud);
    void DrawTrackingPoints(const std::vector<TrackPoint>& trackingPoints, const Pose6dof& pose, const int numExisting);
    void DrawSamplePoints(const std::vector<TrackPoint>& samplePoints);
    void TrimUnusedPoints(std::vector<TrackPoint>& tracks);

    std::vector<TrackPoint> trackingPoints;
    ArrayData<uchar> occupArray;
    int IDcount;

    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullity;
    const AxesType* prinAxes;
    Pose6dof curPose;
};

#endif // POINTTRACKER_H
