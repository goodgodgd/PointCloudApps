#ifndef POINTSAMPLER_H
#define POINTSAMPLER_H

#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/pose6dof.h"
#include "Share/camera_param.h"
#include "IO/drawutils.h"
#include "IO/imageconverter.h"
#include "IO/drawutils.h"
#include "ClUtils/cloperators.h"
#include "ShareExpm/expm_common.h"
#include "ShareExpm/expm_structs.h"


class PointSampler
{
public:
    PointSampler();

    const std::vector<TrackPoint>* SamplePoints(SharedData* shdDat);

private:
    void CollectPoints(std::vector<TrackPoint>& samplePoints);
    void DrawSamplePoints(const std::vector<TrackPoint>& trackingPoints);

    std::vector<TrackPoint> samplePoints;
    const cl_float4* pointCloud;
    const cl_float4* normalCloud;
    const cl_uchar* nullity;
    const AxesType* prinAxes;
};

#endif // POINTSAMPLER_H
