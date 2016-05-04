#include "objectclusterer.h"

ObjectClusterer::ObjectClusterer()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
    , objectMap(nullptr)
{
    objectArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    objectMap = objectArray.GetArrayPtr();
}

void ObjectClusterer::ClusterCloudIntoObjects(SharedData* shdDat)
{
    InitClustering(shdDat);

    vecPairOfInts planeIndexPairs = FindConnectedPlanePairs();

    ClusterPlanes(planeIndexPairs, objectMap, objects);
}

void ObjectClusterer::InitClustering(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();
    planeMap = shdDat->ConstPlaneMap();
    planes = *(shdDat->ConstPlanes());
    memcpy(objectMap, shdDat->ConstPlaneMap(), sizeof(cl_int)*IMAGE_WIDTH*IMAGE_HEIGHT);
    objects = *(shdDat->ConstPlanes());
    imgLines.clear();
    planePairs.clear();
}

vecPairOfInts ObjectClusterer::FindConnectedPlanePairs()
{
    vecPairOfInts pairs;
    typedef vecSegment::const_iterator    vsegIter;
    for(vsegIter ref=planes.begin(); ref+1!=planes.end(); ref++)
    {
        for(vsegIter cmp=ref+1; cmp!=planes.end(); cmp++)
        {
            if(ArePlanesInTheSameObject(*ref, *cmp))
                pairs.emplace_back(ref->id, cmp->id);
        }
    }

    return pairs;
}

bool ObjectClusterer::ArePlanesInTheSameObject(const Segment& leftPlane, const Segment& rightPlane)
{
    if(DoRectsOverlap(leftPlane.rect, rightPlane.rect)==false)
        return false;
    ImRect ovlRect = OverlappingRect(leftPlane.rect, rightPlane.rect);

    vecfPixels connPixels;
    if(ArePlanesConnected(ovlRect, leftPlane, rightPlane, connPixels)==false)
        return false;

    const float similarDegreeRange = 10.f;
    if(clAngleBetweenVectorsLessThan(leftPlane.normal, rightPlane.normal, similarDegreeRange, true))
        return true;

    return ConcaveToEachOther(leftPlane, rightPlane, connPixels);
}

bool ObjectClusterer::DoRectsOverlap(const ImRect& leftRect, const ImRect& rightRect)
{
    if(leftRect.xh <= rightRect.xl || rightRect.xh <= leftRect.xl || leftRect.yh <= rightRect.yl || rightRect.yh <= leftRect.yl)
        return false;
    return true;
}

ImRect ObjectClusterer::OverlappingRect(const ImRect& leftRect, const ImRect& rightRect)
{
    ImRect outRect;
    outRect.xl = smax(leftRect.xl, rightRect.xl);
    outRect.xh = smin(leftRect.xh, rightRect.xh);
    outRect.yl = smax(leftRect.yl, rightRect.yl);
    outRect.yh = smin(leftRect.yh, rightRect.yh);

    outRect.xl = smax(outRect.xl-1, 0);
    outRect.xh = smin(outRect.xh+1, IMAGE_WIDTH-1);
    outRect.yl = smax(outRect.yl-1, 0);
    outRect.yh = smin(outRect.yh+1, IMAGE_HEIGHT-1);
    return outRect;
}

bool ObjectClusterer::ArePlanesConnected(const ImRect& ovlRect, const Segment& leftPlane, const Segment& rightPlane, vecfPixels& connPixels)
{
    int pixidx, rigidx, lowidx;
    int interfaceCount=0, connectCount=0;
    cl_float2 pixel;
    for(int y=ovlRect.yl; y<ovlRect.yh; y++)
    {
        for(int x=ovlRect.xl; x<ovlRect.xh; x++)
        {
            pixidx = IMGIDX(y,x);
            rigidx = IMGIDX(y,x+1);
            lowidx = IMGIDX(y+1,x);

            if((planeMap[pixidx]==leftPlane.id && planeMap[rigidx]==rightPlane.id) || (planeMap[pixidx]==rightPlane.id && planeMap[rigidx]==leftPlane.id))
            {
                interfaceCount++;
                if(ArePixelsConnected(pixidx, leftPlane.normal, rigidx, rightPlane.normal))
                {
                    connectCount++;
                    pixel = (cl_float2){(float)x+0.5f,(float)y};
                    connPixels.push_back(pixel);
                }
            }
            if((planeMap[pixidx]==leftPlane.id && planeMap[lowidx]==rightPlane.id) || (planeMap[pixidx]==rightPlane.id && planeMap[lowidx]==leftPlane.id))
            {
                interfaceCount++;
                if(ArePixelsConnected(pixidx, leftPlane.normal, lowidx, rightPlane.normal))
                {
                    connectCount++;
                    pixel = (cl_float2){(float)x,(float)y+0.5f};
                    connPixels.push_back(pixel);
                }
            }
        }
    }
    return (connectCount > interfaceCount/2 && connectCount > 3);
}

bool ObjectClusterer::ArePixelsConnected(const int leftIdx, const cl_float4& leftNormal, const int rightIdx, const cl_float4& rightNormal)
{
    const float depth = DEPTH(pointCloud[leftIdx]);
    const float depthDiffLimit = smax(depth*depth*0.005f, 0.003f);

    if(fabsf(clDot(pointCloud[leftIdx] - pointCloud[rightIdx], leftNormal)) < depthDiffLimit)
        return true;
    else if(fabsf(clDot(pointCloud[leftIdx] - pointCloud[rightIdx], rightNormal)) < depthDiffLimit)
        return true;

    return false;
}

bool ObjectClusterer::ConcaveToEachOther(const Segment& leftPlane, const Segment& rightPlane, const vecfPixels& connPixels)
{
    connPixels;
    Line2D line = FitLine2D(connPixels);
    imgLines.emplace_back(line.endPixels[0], line.endPixels[1]);
    planePairs.emplace_back(leftPlane.id, rightPlane.id);

    // draw box and 2D line on image


    // find representative points on both plane as far from line as possible
//    PairOfPoints repPoints = PickRepresentPoints(leftPlane, rightPlane);
//    const cl_float4& leftPoint = repPoints.first;
//    const cl_float4& rightPoint = repPoints.second;

    // determine concavity or convexity based on height


    // if concave but height is low, check angle difference between planes


}

Line2D ObjectClusterer::FitLine2D(const vecfPixels& connPixels)
{
    const int itv = smax(connPixels.size()/20, 1);
    ImRect range(connPixels[0].x,connPixels[0].x,connPixels[0].y,connPixels[0].y);
    int szcnt=0;
    for(size_t i=0; i<connPixels.size(); i+=itv)
    {
        range.xl = smin(connPixels[i].x, range.xl);
        range.xh = smax(connPixels[i].x, range.xh);
        range.yl = smin(connPixels[i].y, range.yl);
        range.yh = smax(connPixels[i].y, range.yh);
        szcnt++;
    }
    assert(smax(range.xh - range.xl, range.yh - range.yl) > 1);


    Eigen::MatrixXf A(szcnt, 2);
    Eigen::VectorXf s(2);
    Eigen::VectorXf b(szcnt);
    Line2D line;

    if(range.xh - range.xl > range.yh - range.yl)
    {
        int cnt=0;
        for(size_t i=0; i<connPixels.size(); i+=itv)
        {
            A(cnt,0) = connPixels[i].x;
            A(cnt,1) = 1.f;
            b(cnt) = connPixels[i].y;
            cnt++;
        }
        // y = s(0)*x + s(1)
        s = A.colPivHouseholderQr().solve(b);
        line.a = -s(0);
        line.b = 1.f;
        line.c = s(1);
        line.endPixels[0] = (cl_int2){range.xl, (int)(s(0)*range.xl + s(1))};
        line.endPixels[1] = (cl_int2){range.xh, (int)(s(0)*range.xh + s(1))};
    }
    else
    {
        int cnt=0;
        for(size_t i=0; i<connPixels.size(); i+=itv)
        {
            A(cnt,0) = connPixels[i].y;
            A(cnt,1) = 1.f;
            b(cnt) = connPixels[i].x;
            cnt++;
        }
        // x = s(0)*y + s(1)
        s = A.colPivHouseholderQr().solve(b);
        line.a = 1.f;
        line.b = -s(0);
        line.c = s(1);
        line.endPixels[0] = (cl_int2){(int)(s(0)*range.yl + s(1)), range.yl};
        line.endPixels[1] = (cl_int2){(int)(s(0)*range.yh + s(1)), range.yh};
    }

    return line;
}

PairPointNormal ObjectClusterer::ComputePlaneIntersect(const Segment& leftPlane, const Segment& rightPlane)
{
    const float dL = -clDot(leftPlane.center, leftPlane.normal);
    const float dR = -clDot(rightPlane.center, rightPlane.normal);
    cl_float4 lineDir = clCross(leftPlane.normal, rightPlane.normal);
    cl_float4 subs = leftPlane.normal*dR - rightPlane.normal*dL;
    cl_float4 linePoint = clCross(subs, lineDir) / clSqLength(lineDir);
    lineDir = clNormalize(lineDir);
    PairPointNormal line(linePoint, lineDir);

    assert(TestPlaneIntersect(leftPlane, rightPlane, line));
    return line;
}

PairOfPixels ObjectClusterer::ConvertToImageLine(const PairPointNormal& intersectLine, const ImRect& ovlRect)
{
    const cl_float4& linePoint = intersectLine.first;
    const cl_float4& lineDir = intersectLine.second;
    cl_float2 firstPixel = ProjectPointOntoImage(linePoint);
    cl_float2 secondPixel = ProjectPointOntoImage(linePoint + lineDir);
    // ax + by = c
    float a = (secondPixel.y - firstPixel.y);
    float b = -(secondPixel.x - firstPixel.x);
    float c = (firstPixel.x*secondPixel.y - secondPixel.x*firstPixel.y);
    const bool majorImgAxisX = (fabsf(a) < fabsf(b));

    if(majorImgAxisX)
    {
        int yforxl = (int)(-ovlRect.xl*a/b + c/b);
        int yforxh = (int)(-ovlRect.xh*a/b + c/b);
//        qDebug() << "Xm Line3D" << linePoint << lineDir << "imgline" << ovlRect.xl << yforxl << ovlRect.xh << yforxh;
        return std::make_pair((cl_int2){ovlRect.xl, yforxl}, (cl_int2){ovlRect.xh, yforxh});
    }
    else
    {
        int xforyl = (int)(-ovlRect.yl*b/a + c/a);
        int xforyh = (int)(-ovlRect.yh*b/a + c/a);
//        qDebug() << "Ym Line3D" << linePoint << lineDir << "imgline" << xforyl << ovlRect.yl << xforyh << ovlRect.yh;
        return std::make_pair((cl_int2){xforyl, ovlRect.yl}, (cl_int2){xforyh, ovlRect.yh});
    }
}

cl_float2 ObjectClusterer::ProjectPointOntoImage(const cl_float4& srcpt)
{
    return (cl_float2){-srcpt.y/DEPTH(srcpt)*FOCAL_LENGTH + IMAGE_WIDTH/2.f
                        , -srcpt.z/DEPTH(srcpt)*FOCAL_LENGTH + IMAGE_HEIGHT/2.f};
}

void ObjectClusterer::ClusterPlanes(const vecPairOfInts& pairs, cl_int* objectMap, vecSegment& objects)
{
}

void ObjectClusterer::MergePlanesConcaveToEachOther()
{
}

const cl_int* ObjectClusterer::GetObjectMap()
{
    return objectMap;
}

const vecSegment* ObjectClusterer::GetObjects()
{
    return &objects;
}

