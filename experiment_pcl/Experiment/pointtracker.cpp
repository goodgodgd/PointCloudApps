#include "pointtracker.h"

PointTracker::PointTracker()
    : occupArray(IMAGE_WIDTH*IMAGE_HEIGHT)
    , IDcount(0)
{
}

const std::vector<TrackPoint>* PointTracker::SamplePoints(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullity = shdDat->ConstNullityMap();
    curPose = shdDat->ConstGlobalPose();
    prinAxes = shdDat->ConstPrinAxes();

    trackingPoints.clear();
    occupArray.SetZero();
    AppendNewTracks(trackingPoints);
    DrawSamplePoints(trackingPoints);
    return &trackingPoints;
}

const std::vector<TrackPoint>* PointTracker::Track(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullity = shdDat->ConstNullityMap();
    curPose = shdDat->ConstGlobalPose();
    prinAxes = shdDat->ConstPrinAxes();

    static int befFrameIdx = 10000;
    if(g_frameIdx < befFrameIdx)
        trackingPoints.clear();
    befFrameIdx = g_frameIdx;

    int trackCount = TrackPoints(trackingPoints);
    int numTracking = trackingPoints.size();
    qDebug() << "tracking" << trackCount << "outof" << trackingPoints.size();

    TrimUnusedPoints(trackingPoints);

    if(g_frameIdx%5==1)
    {
        AppendNewTracks(trackingPoints);
        qDebug() << "tracks added" << trackingPoints.size();
    }

    DrawTrackingPoints(trackingPoints, curPose, numTracking);
    return &trackingPoints;
}

int PointTracker::TrackPoints(std::vector<TrackPoint>& srcPoints)
{
    cl_uint2 pixel;
    occupArray.SetZero();
    uchar* occpMap = occupArray.GetArrayPtr();
    int trackCount=0;

    for(TrackPoint& spoint : srcPoints)
    {
        try {
            pixel = SearchCorrespondingPixel(spoint);
            if(DEPTH(pointCloud[PIXIDX(pixel)]) > CameraParam::trackRange())
                continue;
            UpdateTrackPoint(spoint, pixel);
            occpMap[PIXIDX(spoint.pixel)] = 1;
            trackCount++;
        }
        catch(TrackException exception) {
            qDebug() << "TrackException:" << exception.msg;
        }
        catch(EmtpyException code) { /*qDebug() << "Empty";*/ }
    }
    return trackCount;
}

cl_uint2 PointTracker::SearchCorrespondingPixel(const TrackPoint& srcPoint)
{
    const cl_uint2 prjPixel = ProjectOntoImage(srcPoint.gpoint, curPose);
    const int margin = 2;
    if(prjPixel.x < margin || prjPixel.x > IMAGE_WIDTH-margin-1 || prjPixel.y < margin || prjPixel.y > IMAGE_HEIGHT-margin-1)
        throw EmtpyException(0);
    cl_float4 pixGPoint, pixGNormal;
    cl_uint2 crpPixel;
    const float pointDistUpLimit = DEPTH(pointCloud[PIXIDX(prjPixel)])*0.01f;
    const float normalAngleDiffLimit = 15.f;
    const float normalAngleWeight = 0.01f/5.f;
    float ptdist, angle, minDist = 1.f;
    float minPtdist=1.f, minAngle=1.f;

    for(int y=(int)prjPixel.y-margin; y<=(int)prjPixel.y+margin; y++)
    {
        for(int x=(int)prjPixel.x-margin; x<=(int)prjPixel.x+margin; x++)
        {
            if(nullity[IMGIDX(y,x)] != NullID::NoneNull)
                continue;

            pixGPoint = curPose.Local2Global(pointCloud[IMGIDX(y,x)]);
            pixGNormal = curPose.Rotate2Global(normalCloud[IMGIDX(y,x)]);

            ptdist = clLength(srcPoint.gpoint - pixGPoint);
            angle = clAngleBetweenVectorsDegree(srcPoint.gnormal, pixGNormal, false);

            if(ptdist + angle*normalAngleWeight < minDist)
            {
                minDist = ptdist + angle*normalAngleWeight;
                crpPixel = (cl_uint2){x, y};
                minPtdist = ptdist;
                minAngle = angle;
            }
        }
    }

    if(minPtdist > pointDistUpLimit || minAngle > normalAngleDiffLimit)
//        throw TrackException(QString("no close point to (%1 %2 %3) %4").arg(srcPoint.gpoint.x).arg(srcPoint.gpoint.y).arg(srcPoint.gpoint.z).arg(minDist));
        throw EmtpyException(0);

    return crpPixel;
}

cl_uint2 PointTracker::ProjectOntoImage(const cl_float4& gpoint, const Pose6dof& pose)
{
    const cl_float4 lpoint = pose.Global2Local(gpoint);
    if(lpoint.x < CameraParam::RangeBeg_m() || lpoint.x > CameraParam::RangeEnd_m())
//        throw TrackException(QString("the point is behind camera (%1 %2 %3)").arg(lpoint.x).arg(lpoint.y).arg(lpoint.z));
        throw EmtpyException(0);

    const int margin = 2;
    cl_float2 fpixel = ImageConverter::PointToPixel(lpoint);
    cl_uint2 pixel = (cl_uint2){(int)(fpixel.x+0.5f), (int)(fpixel.y-0.5f)};
    if(pixel.x < margin || pixel.x >= IMAGE_WIDTH-margin || pixel.y < margin || pixel.y >= IMAGE_HEIGHT-margin)
//        throw TrackException(QString("out of fov (%1 %2)").arg(pixel.x).arg(pixel.y));
        throw EmtpyException(0);

    return pixel;
}

void PointTracker::UpdateTrackPoint(TrackPoint& srcPoint, const cl_uint2& selPixel)
{
    srcPoint.ID = srcPoint.ID;
    srcPoint.pixel = selPixel;
    srcPoint.frameIndex = g_frameIdx;
//    srcPoint.gpoint = (srcPoint.gpoint*(float)srcPoint.tcount + curPose.Local2Global(pointCloud[PIXIDX(selPixel)]))/(float)(srcPoint.tcount + 1);
//    srcPoint.gnormal = (srcPoint.gnormal*(float)srcPoint.tcount + curPose.Rotate2Global(normalCloud[PIXIDX(selPixel)]));
//    srcPoint.gnormal = clNormalize(srcPoint.gnormal);
    srcPoint.tcount++;
}

void PointTracker::AppendNewTracks(std::vector<TrackPoint>& trackPoints)
{
    uchar* occpMap = occupArray.GetArrayPtr();
    const int interval = 2;
    const int offset = interval/2;
    cl_uint2 pixel;
    TrackPoint trackpt;
    cl_uint pxidx;

    for(int y=offset; y<IMAGE_HEIGHT-offset; y+=interval)
    {
        for(int x=offset; x<IMAGE_WIDTH-offset; x+=interval)
        {
            pixel = (cl_uint2){x,y};
            pxidx = IMGIDX(y,x);
            if(nullity[pxidx] != NullID::NoneNull)
                continue;
            if(DEPTH(pointCloud[pxidx]) > CameraParam::sampleRange())
                continue;
            if(FindAdjacentPoint(pixel, occpMap, pointCloud, normalCloud))
                continue;

            trackpt.ID = ++IDcount;
            trackpt.beginIndex = g_frameIdx;
            trackpt.frameIndex = g_frameIdx;
            trackpt.pixel = pixel;
            trackpt.gpoint = pointCloud[pxidx];
            trackpt.gnormal = normalCloud[pxidx];
            trackpt.tcount = 1;
            trackPoints.push_back(trackpt);
            occpMap[pxidx] = 1;
        }
    }
}

bool PointTracker::FindAdjacentPoint(const cl_uint2& pixel, const uchar* occpMap, const cl_float4* pointCloud, const cl_float4* normalCloud)
{
    const cl_float4& herePoint = pointCloud[PIXIDX(pixel)];
    const cl_float4& hereNormal = normalCloud[PIXIDX(pixel)];
    const float minDist = 0.03f;
    const float searchRadius = 0.10f;
    const int pixelRadius = smax(searchRadius/DEPTH(herePoint)*CameraParam::flh(), 4.f);
    ImRect searchRect(smax(0, pixel.x-pixelRadius), smin(IMAGE_WIDTH-1, pixel.x+pixelRadius)
                      , smax(0, pixel.y-pixelRadius), smin(IMAGE_HEIGHT-1, pixel.y+pixelRadius));

    for(int y=searchRect.yl; y<=searchRect.yh; y++)
    {
        for(int x=searchRect.xl; x<=searchRect.xh; x++)
        {
            if(occpMap[IMGIDX(y,x)]==0)
                continue;

            if(clAngleBetweenVectorsLessThan(hereNormal, normalCloud[IMGIDX(y,x)], 20, true))
                return true;
            if(clLength(herePoint - pointCloud[IMGIDX(y,x)]) < minDist)
                return true;
        }
    }
    return false;
}

void PointTracker::DrawTrackingPoints(const std::vector<TrackPoint>& trackingPoints, const Pose6dof& pose, const int numExisting)
{
    const QRgb rgbTrack = qRgb(0,0,255);
    const QRgb rgbExist = qRgb(0,255,0);
    const QRgb rgbNew = qRgb(255,0,0);
    cl_uint2 pixel;
    cl_float4 point, normal;

    for(int i=0; i<trackingPoints.size(); i++)
    {
        pixel = trackingPoints[i].pixel;
        point = pose.Global2Local(trackingPoints[i].gpoint);
        normal = pose.Rotate2Local(trackingPoints[i].gnormal);
        if(i<numExisting && trackingPoints[i].frameIndex==g_frameIdx)
            DrawUtils::MarkPoint3D(point, normal, rgbTrack, 0.05f);
        else if(i<numExisting)
            DrawUtils::MarkPoint3D(point, normal, rgbExist, 0.05f);
        else
            DrawUtils::MarkPoint3D(point, normal, rgbNew, 0.05f);
    }
}

void PointTracker::DrawSamplePoints(const std::vector<TrackPoint>& samplePoints)
{
    const QRgb rgbNew = qRgb(255,0,0);
    for(int i=0; i<samplePoints.size(); i++)
        DrawUtils::MarkPoint3D(samplePoints[i].gpoint, samplePoints[i].gnormal, rgbNew, 0.02f);
}

void PointTracker::TrimUnusedPoints(std::vector<TrackPoint>& tracks)
{
    for(size_t i=0; i<tracks.size(); ++i)
    {
        if(g_frameIdx - tracks[i].beginIndex > 10 && tracks[i].tcount < 3)
        {
            tracks.erase(tracks.begin()+i);
            --i;
        }
    }
}
