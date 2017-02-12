#include "pointsampler.h"

PointSampler::PointSampler()
{
}

const std::vector<TrackPoint>* PointSampler::SamplePoints(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullity = shdDat->ConstNullityMap();
    prinAxes = shdDat->ConstPrinAxes();

    samplePoints.clear();
    CollectPoints(samplePoints);
    DrawSamplePoints(samplePoints);
    return &samplePoints;
}

void PointSampler::CollectPoints(std::vector<TrackPoint>& samplePoints)
{
    const int stride = 10;
    int pxidx;
    TrackPoint sample;
    int IDCount=0;
    cl_float4 zerovec = (cl_float4){0,0,0,0};

    for(int y=0; y<IMAGE_HEIGHT; y+=stride)
    {
        for(int x=0; x<IMAGE_WIDTH; x+=stride)
        {
            pxidx = IMGIDX(y,x);
            if(nullity[pxidx] != NullID::NoneNull)
                continue;
            if(DEPTH(pointCloud[pxidx]) > CameraParam::sampleRange())
                continue;

            sample.ID = IDCount++;
            sample.pixel = (cl_uint2){x,y};
            sample.beginIndex = g_frameIdx;
            sample.frameIndex = g_frameIdx;
            sample.tcount = 1;
            sample.gnormal = zerovec;
            sample.gpoint = zerovec;
            samplePoints.push_back(sample);
        }
    }
}

void PointSampler::DrawSamplePoints(const std::vector<TrackPoint>& trackingPoints)
{
    const QRgb rgbTrack = qRgb(255,0,0);
    cl_float4 point, normal;

    for(int i=0; i<trackingPoints.size(); i++)
    {
        cl_uint2 pixel = trackingPoints[i].pixel;
        point = pointCloud[PIXIDX(pixel)];
        normal = normalCloud[PIXIDX(pixel)];
        DrawUtils::MarkPoint3D(point, normal, rgbTrack, 0.02f);
    }
}
