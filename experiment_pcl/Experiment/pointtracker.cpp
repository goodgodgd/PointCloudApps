#include "pointtracker.h"

PointTracker::PointTracker()
    : occupArray(IMAGE_WIDTH*IMAGE_HEIGHT)
    , IDcount(0)
{
}

const std::vector<TrackPoint>* PointTracker::Track(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullity = shdDat->ConstNullityMap();
    curPose = shdDat->ConstGlobalPose();

    std::vector<TrackPoint> trackedPoints = TrackPoints(trackingPoints);
    trackingPoints.swap(trackedPoints);
    NumExisting = trackingPoints.size();

    std::vector<TrackPoint> neoPoints = SampleNewPoints();
    trackingPoints.insert(trackingPoints.end(), neoPoints.begin(), neoPoints.end());
    qDebug() << "track" << NumExisting << neoPoints.size() << trackingPoints.size();

    DrawTrackingPoints(trackingPoints, NumExisting);

    return &trackingPoints;
}

std::vector<TrackPoint> PointTracker::TrackPoints(const std::vector<TrackPoint>& srcPoints)
{
    std::vector<TrackPoint> dstPoints;
    TrackPoint dpoint;
    cl_uint2 pixel;
    occupArray.SetZero();
    uchar* occpMap = occupArray.GetArrayPtr();

    for(TrackPoint spoint : srcPoints)
    {
        try {
            pixel = SearchCorrespondingPixel(spoint);
            dpoint = UpdateTrackPoint(spoint, pixel);
            dstPoints.push_back(dpoint);

            occpMap[PIXIDX(dpoint.pixel)] = 1;
        }
        catch(TrackException exception) {
            qDebug() << "TrackException:" << exception.msg;
        }
        catch(int code) {}
    }

    return dstPoints;
}

cl_uint2 PointTracker::SearchCorrespondingPixel(const TrackPoint& srcPoint)
{
    const cl_uint2 prjPixel = ProjectOntoImage(srcPoint.gpoint, curPose);
    cl_float4 pixGPoint, pixGNormal;
    cl_uint2 crpPixel;
    const float pointDistUpLimit = DEPTH(pointCloud[PIXIDX(prjPixel)])*0.01f;
    const float normalDistUpLimit = DEPTH(pointCloud[PIXIDX(prjPixel)])*0.05f;
    const float normalDistWeight = 0.2f;
    float ptdist, nmdist, minDist = 1.f;

    for(int y=(int)prjPixel.y-1; y<=(int)prjPixel.y+1; y++)
    {
        for(int x=(int)prjPixel.x-1; x<=(int)prjPixel.x+1; x++)
        {
            if(nullity[IMGIDX(y,x)] != NullID::NoneNull)
                continue;

            pixGPoint = curPose.Local2Global(pointCloud[IMGIDX(y,x)]);
            pixGNormal = curPose.Rotate2Global(normalCloud[IMGIDX(y,x)]);

            ptdist = clLength(srcPoint.gpoint - pixGPoint);
            nmdist = clLength(srcPoint.gnormal - pixGNormal);

//            if(srcPoint.ID==trackID && prjPixel.x==x && prjPixel.y==y)
//            {
//                qDebugPrec(3) << "track" << srcPoint.ID << srcPoint.pixel << "cur" << prjPixel << (cl_uint2){x, y};
//                qDebugPrec(4) << "  " << pointCloud[IMGIDX(y,x)] << pixGPoint << srcPoint.gpoint << ptdist;
//                qDebugPrec(4) << "  " << normalCloud[IMGIDX(y,x)] << pixGNormal << srcPoint.gnormal << nmdist;
//            }

            if(ptdist > pointDistUpLimit || nmdist > normalDistUpLimit)
                continue;

            if(ptdist + nmdist*normalDistWeight < minDist)
            {
                minDist = ptdist + nmdist*normalDistWeight;
                crpPixel = (cl_uint2){x, y};
            }
        }
    }

    if(minDist > 0.1f)
//        throw TrackException(QString("no close point to (%1 %2 %3) %4").arg(srcPoint.gpoint.x).arg(srcPoint.gpoint.y).arg(srcPoint.gpoint.z).arg(minDist));
        throw 0;

    return crpPixel;
}

cl_uint2 PointTracker::ProjectOntoImage(const cl_float4& gpoint, const Pose6dof& pose)
{
    const cl_float4 lpoint = pose.Global2Local(gpoint);
    if(lpoint.x < CameraParam::RangeBeg_m() || lpoint.x > CameraParam::RangeEnd_m())
//        throw TrackException(QString("the point is behind camera (%1 %2 %3)").arg(lpoint.x).arg(lpoint.y).arg(lpoint.z));
        throw 0;

    const int margin = 2;
    cl_float2 fpixel = ImageConverter::PointToPixel(lpoint);
    cl_uint2 pixel = (cl_uint2){(int)(fpixel.x+0.5f), (int)(fpixel.y-0.5f)};
    if(pixel.x < margin || pixel.x >= IMAGE_WIDTH-margin || pixel.y < margin || pixel.y >= IMAGE_HEIGHT-margin)
//        throw TrackException(QString("out of fov (%1 %2)").arg(pixel.x).arg(pixel.y));
        throw 0;

    return pixel;
}

TrackPoint PointTracker::UpdateTrackPoint(const TrackPoint& srcPoint, const cl_uint2& selPixel)
{
    TrackPoint dpoint;
    dpoint.ID = srcPoint.ID;
    dpoint.pixel = selPixel;
    dpoint.frameIndex = g_frameIdx;
    dpoint.lpoint = pointCloud[PIXIDX(selPixel)];
    dpoint.tcount = srcPoint.tcount + 1;

    dpoint.gpoint = (srcPoint.gpoint*(float)srcPoint.tcount + curPose.Local2Global(pointCloud[PIXIDX(selPixel)]))/(float)dpoint.tcount;
    dpoint.gnormal = (srcPoint.gnormal*(float)srcPoint.tcount + curPose.Rotate2Global(normalCloud[PIXIDX(selPixel)]));
    dpoint.gnormal = clNormalize(dpoint.gnormal);

    return dpoint;
}

std::vector<TrackPoint> PointTracker::SampleNewPoints()
{
    std::vector<TrackPoint> neoPoints;
    const uchar* occpMap = occupArray.GetArrayPtr();
    const int interval = 5;
    const int offset = interval/2;
    cl_uint2 pixel;
    TrackPoint trackpt;

    for(int y=offset; y<IMAGE_HEIGHT-offset; y+=interval)
    {
        for(int x=offset; x<IMAGE_WIDTH-offset; x+=interval)
        {
            pixel = (cl_uint2){x,y};
            if(nullity[IMGIDX(y,x)] != NullID::NoneNull)
                continue;

            if(FindAdjacentPoint(pixel, occpMap, pointCloud)==false)
            {
                trackpt.ID = ++IDcount;
                trackpt.frameIndex = g_frameIdx;
                trackpt.pixel = pixel;
                trackpt.lpoint = pointCloud[IMGIDX(y,x)];
                trackpt.gpoint = curPose.Local2Global(pointCloud[IMGIDX(y,x)]);
                trackpt.gnormal = curPose.Rotate2Global(normalCloud[IMGIDX(y,x)]);
                trackpt.tcount = 1;
                neoPoints.push_back(trackpt);
            }
        }
    }

    return neoPoints;
}

bool PointTracker::FindAdjacentPoint(const cl_uint2& pixel, const uchar* occpMap, const cl_float4* pointCloud)
{
    const cl_float4& herePoint = pointCloud[PIXIDX(pixel)];
    const float radius = 0.05f;
    const int pixelRadius = smax(radius/herePoint.x*CameraParam::flh(), 4.f);
    ImRect searchRect(smax(0, pixel.x-pixelRadius), smin(IMAGE_WIDTH-1, pixel.x+pixelRadius)
                      , smax(0, pixel.y-pixelRadius), smin(IMAGE_HEIGHT-1, pixel.y+pixelRadius));

    for(int y=searchRect.yl; y<=searchRect.yh; y++)
    {
        for(int x=searchRect.xl; x<=searchRect.xh; x++)
        {
            if(occpMap[IMGIDX(y,x)]==0)
                continue;

            if(clLength(herePoint - pointCloud[IMGIDX(y,x)]) < radius)
                return true;
        }
    }
    return false;
}

void PointTracker::DrawTrackingPoints(const std::vector<TrackPoint>& trackingPoints, const int NumExisting)
{
    const QRgb rgbExist = qRgb(0,0,255);
    const QRgb rgbNew = qRgb(255,0,0);
    cl_uint2 pixel;

    for(int i=0; i<trackingPoints.size(); i++)
    {
        pixel = trackingPoints[i].pixel;
        if(i<NumExisting)
            DrawUtils::MarkPoint3D(pointCloud[PIXIDX(pixel)], normalCloud[PIXIDX(pixel)], rgbExist, 0.02f);
        else
            DrawUtils::MarkPoint3D(pointCloud[PIXIDX(pixel)], normalCloud[PIXIDX(pixel)], rgbNew, 0.02f);
    }
}
