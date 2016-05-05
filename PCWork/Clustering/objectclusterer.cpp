#include "objectclusterer.h"

int dbgFirstId = 0;
int dbgSecondId = 0;

ObjectClusterer::ObjectClusterer()
    : pointCloud(nullptr)
    , normalCloud(nullptr)
    , nullityMap(nullptr)
    , planeMap(nullptr)
    , objectMap(nullptr)
{
    objectArray.Allocate(IMAGE_WIDTH*IMAGE_HEIGHT);
    objectMap = objectArray.GetArrayPtr();
}

void ObjectClusterer::ClusterCloudIntoObjects(SharedData* shdDat)
{
    InitClustering(shdDat);

    for(auto ref=planes.begin(); ref+1!=planes.end(); ++ref)
    {
        if(ref->id==MERGED_PLANE)
            continue;
        for(auto cmp=planes.rbegin(); cmp->id!=ref->id; ++cmp)
        {
            if(cmp->id==MERGED_PLANE)
                continue;
            if(ArePlanesInTheSameObject(*ref, *cmp))
                MergePlanes(*ref, *cmp);
        }
    }

    ExtractValidSegments(planes, objects);
}

void ObjectClusterer::InitClustering(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();
    planeMap = shdDat->ConstPlaneMap();
    planes = *(shdDat->ConstPlanes());
    memcpy(objectMap, shdDat->ConstPlaneMap(), sizeof(cl_int)*IMAGE_WIDTH*IMAGE_HEIGHT);
    borderLines.clear();
    IdPairs.clear();

    std::sort(planes.begin(), planes.end(), [](const Segment& a, const Segment& b)
                                              { return a.numpt > b.numpt; });
    qDebug() << "sort result" << planes[0].numpt << planes[1].numpt << planes[2].numpt;
}

void ObjectClusterer::ExtractValidSegments(const vecSegment& planes, vecSegment& objects)
{
    for(auto& plane : planes)
    {
        if(plane.id!=MERGED_PLANE)
        {
            mapIdIndex[plane.id] = objects.size();
            objects.push_back(plane);
        }
    }
}

bool ObjectClusterer::ArePlanesInTheSameObject(const Segment& largerPlane, const Segment& smallerPlane)
{
    dbgFirstId = largerPlane.id;
    dbgSecondId = smallerPlane.id;
    assert(largerPlane.numpt >= smallerPlane.numpt);
    if(DoRectsOverlap(largerPlane.rect, smallerPlane.rect)==false)
        return false;
    ImRect ovlRect = OverlappingRect(largerPlane.rect, smallerPlane.rect);

    vecPairOfPixels connPixels;
    if(ArePlanesConnected(ovlRect, largerPlane, smallerPlane, connPixels)==false)
        return false;

    if(clAngleBetweenVectorsLessThan(largerPlane.normal, smallerPlane.normal, 10.f, true))
        return true;

    if(smallerPlane.numpt < 50)
        return IncludeTinyPlane(largerPlane, smallerPlane);

    return !ConvexToEachOther(largerPlane, smallerPlane, connPixels);
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

bool ObjectClusterer::ArePlanesConnected(const ImRect& ovlRect, const Segment& firstPlane, const Segment& secondPlane, vecPairOfPixels& connPixels)
{
    int pixidx, rigidx, lowidx;
    int interfaceCount=0;
    connPixels.clear();

    for(int y=ovlRect.yl; y<ovlRect.yh; y++)
    {
        for(int x=ovlRect.xl; x<ovlRect.xh; x++)
        {
            pixidx = IMGIDX(y,x);
            rigidx = IMGIDX(y,x+1);
            lowidx = IMGIDX(y+1,x);

            if(planeMap[pixidx]==firstPlane.id && planeMap[rigidx]==secondPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(pixidx, firstPlane.normal, rigidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x,y},(cl_int2){x+1,y});
            }
            if(planeMap[pixidx]==secondPlane.id && planeMap[rigidx]==firstPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(rigidx, firstPlane.normal, pixidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x+1,y}, (cl_int2){x,y});
            }

            if(planeMap[pixidx]==firstPlane.id && planeMap[lowidx]==secondPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(pixidx, firstPlane.normal, lowidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x,y},(cl_int2){x,y+1});
            }
            if(planeMap[pixidx]==secondPlane.id && planeMap[lowidx]==firstPlane.id)
            {
                interfaceCount++;
                if(ArePixelsConnected(lowidx, firstPlane.normal, pixidx, secondPlane.normal))
                    connPixels.emplace_back((cl_int2){x,y+1}, (cl_int2){x,y});
            }
        }
    }
    return (connPixels.size() > interfaceCount/2 && connPixels.size() > 3);
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

bool ObjectClusterer::IncludeTinyPlane(const Segment& largerPlane, const Segment& smallerPlane)
{
    assert(clDot(largerPlane.center, largerPlane.normal) < 0.f);
    assert(fabsf(clLength(largerPlane.normal)-1.f) < 0.0001f);

    const float planeDist = -clDot(largerPlane.center, largerPlane.normal);
    const float heightUpLimit = smax(planeDist*planeDist*0.01f, 0.005f);
    float minNormalDist=10000.f;
    const ImRect& range = smallerPlane.rect;
    for(int y=range.yl; y<=range.yh; y++)
    {
        for(int x=range.xl; x<=range.xh; x++)
        {
            if(planeMap[IMGIDX(y,x)]==smallerPlane.id)
                minNormalDist = smin(minNormalDist, -clDot(pointCloud[IMGIDX(y,x)], largerPlane.normal));
        }
    }
    float maxHeightFromLargePlane = planeDist - minNormalDist;
//    qDebug() << "includeTiny" << largerPlane.id << largerPlane.numpt << smallerPlane.id << smallerPlane.numpt
//             << "height" << planeDist << planeDist - minNormalDist << heightUpLimit << (planeDist - minNormalDist < heightUpLimit);

    // include tiny neighbor plane except for hightly convex to large plane
    return (maxHeightFromLargePlane < heightUpLimit);
}

void ObjectClusterer::MergePlanes(Segment& largerPlane, Segment& smallerPlane)
{
    for(int y=smallerPlane.rect.yl; y<=smallerPlane.rect.yh; y++)
    {
        for(int x=smallerPlane.rect.xl; x<=smallerPlane.rect.xh; x++)
        {
            if(objectMap[IMGIDX(y,x)]==smallerPlane.id)
                objectMap[IMGIDX(y,x)] = largerPlane.id;
        }
    }
    smallerPlane.id = MERGED_PLANE;
    largerPlane.numpt += smallerPlane.numpt;
    largerPlane.rect.ExpandRange(smallerPlane.rect);
}

bool ObjectClusterer::ConvexToEachOther(const Segment& firstPlane, const Segment& secondPlane, const vecPairOfPixels& connPixels)
{
    ImLine border = FitLine2D(connPixels);
    IdPairs.emplace_back(firstPlane.id, secondPlane.id);
    borderLines.push_back(border);

    bool firstPlaneUp = IsFirstUpperPlane(border, connPixels);
    if(fabsf(border.a) < fabsf(border.b))
        qDebug() << "X major" << firstPlane.id << secondPlane.id << firstPlaneUp
                 << (firstPlane.rect.yl+firstPlane.rect.yh)/2 << (secondPlane.rect.yl+secondPlane.rect.yh)/2
                 << (firstPlane.rect.yl+firstPlane.rect.yh > secondPlane.rect.yl+secondPlane.rect.yh)
                 << firstPlane.rect << secondPlane.rect;
    else
        qDebug() << "Y major" << firstPlane.id << secondPlane.id << firstPlaneUp
                 << (firstPlane.rect.xl+firstPlane.rect.xh)/2 << (secondPlane.rect.xl+secondPlane.rect.xh)/2
                 << (firstPlane.rect.xl+firstPlane.rect.xh > secondPlane.rect.xl+secondPlane.rect.xh)
                 << firstPlane.rect << secondPlane.rect;

    cl_float4 borderCenter = PickCenterPointOfBorder(border);

    return true;

    cl_float4 pointOnFirst = ComputeVirtualPoints(firstPlane, border, firstPlaneUp);
    cl_float4 pointOnSecond = ComputeVirtualPoints(secondPlane, border, !firstPlaneUp);

    float innerAngle = InnerAngleDegree(borderCenter, pointOnFirst, pointOnSecond);
    if(innerAngle > 170.f)
        return false;
    else if(innerAngle > 160.f)
        return DetermineConvexityByHeight(firstPlane, secondPlane);
    else
        return true;
}

ImLine ObjectClusterer::FitLine2D(const vecPairOfPixels& connPixels)
{
    const int itv = smax(connPixels.size()/20, 1);
    ImRect range(connPixels[0].first.x,connPixels[0].first.x,connPixels[0].first.y,connPixels[0].first.y);
    int szcnt=0;
    for(size_t i=0; i<connPixels.size(); i+=itv)
    {
        range.ExpandRange(connPixels[i].first);
        range.ExpandRange(connPixels[i].second);
        szcnt++;
    }
    assert(smax(range.xh - range.xl, range.yh - range.yl) > 1);

    Eigen::MatrixXf A(szcnt, 2);
    Eigen::VectorXf s(2);
    Eigen::VectorXf b(szcnt);
    ImLine line;

    if(range.xh - range.xl > range.yh - range.yl)
    {
        int cnt=0;
        for(size_t i=0; i<connPixels.size(); i+=itv)
        {
            A(cnt,0) = (connPixels[i].first.x + connPixels[i].second.x)/2.f;
            A(cnt,1) = 1.f;
            b(cnt) = (connPixels[i].first.y + connPixels[i].second.y)/2.f;
            cnt++;
        }
        // y = s(0)*x + s(1) => -s(0)*x + y = s(1)
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
            A(cnt,0) = (connPixels[i].first.y + connPixels[i].second.y)/2.f;
            A(cnt,1) = 1.f;
            b(cnt) = (connPixels[i].first.x + connPixels[i].second.x)/2.f;
            cnt++;
        }
        // x = s(0)*y + s(1) => x + -s(0)*y = s(1)
        s = A.colPivHouseholderQr().solve(b);
        line.a = 1.f;
        line.b = -s(0);
        line.c = s(1);
        line.endPixels[0] = (cl_int2){(int)(s(0)*range.yl + s(1)), range.yl};
        line.endPixels[1] = (cl_int2){(int)(s(0)*range.yh + s(1)), range.yh};
    }
    return line;
}

bool ObjectClusterer::IsFirstUpperPlane(const ImLine& border, const vecPairOfPixels& connPixels)
{
    float firstSum=0.f, secondSum=0.f;
    for(auto& pixelPair : connPixels)
    {
        firstSum += border.a*pixelPair.first.x + border.b*pixelPair.first.y;
        secondSum += border.a*pixelPair.second.x + border.b*pixelPair.second.y;
    }
    return (firstSum > secondSum);
}

cl_float4 ObjectClusterer::PickCenterPointOfBorder(const ImLine& border)
{
    cl_int2 centerPixel = (border.endPixels[0] + border.endPixels[1])/2;
    if(nullityMap[PIXIDX(centerPixel)] >= NullID::PointNull)
    {
        if(fabsf(border.b) > fabsf(border.a))
            centerPixel = SearchValidPixelOnLine(border, centerPixel, 0, 1);
        else
            centerPixel = SearchValidPixelOnLine(border, centerPixel, 1, 0);
        qDebug() << "   Null Center" << centerPixel << (fabsf(border.b) > fabsf(border.a)) << pointCloud[PIXIDX(centerPixel)];
    }
    return pointCloud[PIXIDX(centerPixel)];
}

cl_int2 ObjectClusterer::SearchValidPixelOnLine(const ImLine& border, const cl_int2& centerPixel, const int majorAxis, const int minorAxis)
{
    assert(majorAxis*minorAxis==0 && majorAxis+minorAxis==1);
    cl_int2 linePixel = centerPixel;
    int move=1;
    int rangeMin = smin(border.endPixels[0].s[majorAxis], border.endPixels[1].s[majorAxis]);
    int rangeMax = smax(border.endPixels[0].s[majorAxis], border.endPixels[1].s[majorAxis]);
    cl_float4 coef = (cl_float4){border.a, border.b, border.c, 0.f};
    while(linePixel.s[majorAxis] >= rangeMin && linePixel.s[majorAxis] <= rangeMax)
    {
        linePixel.s[majorAxis] = centerPixel.s[majorAxis]-move;
        linePixel.s[minorAxis] = (int)(-coef.s[majorAxis]/coef.s[minorAxis]*linePixel.s[majorAxis] + coef.s[2]/coef.s[minorAxis]);
        if(nullityMap[PIXIDX(linePixel)] < NullID::PointNull)
            return linePixel;
        linePixel.s[majorAxis] = centerPixel.s[majorAxis]+move;
        linePixel.s[minorAxis] = (int)(-coef.s[majorAxis]/coef.s[minorAxis]*linePixel.s[majorAxis] + coef.s[2]/coef.s[minorAxis]);
        if(nullityMap[PIXIDX(linePixel)] < NullID::PointNull)
            return linePixel;
    }
    return centerPixel;
}

cl_float4 ObjectClusterer::ComputeVirtualPoints(const Segment& plane, const ImLine& border, bool fisrtPlaneUp)
{

}

float ObjectClusterer::InnerAngleDegree(const cl_float4& borderCenter, const cl_float4& pointOnFirst, const cl_float4& pointOnSecond)
{

}

bool ObjectClusterer::DetermineConvexityByHeight(const Segment& firstPlane, const Segment& secondPlane)
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

const Segment* ObjectClusterer::GetObjectByID(const int ID)
{
    if(mapIdIndex.find(ID)!=mapIdIndex.end())
        return &objects[mapIdIndex[ID]];
    else
        return nullptr;
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

