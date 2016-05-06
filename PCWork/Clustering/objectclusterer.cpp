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

    AbsorbTinyPlanes();

    MergeLargePlanes();

    ExtractValidSegments(planes, objects);
    qDebug() << "objects" << objects.size();
}

void ObjectClusterer::InitClustering(SharedData* shdDat)
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    nullityMap = shdDat->ConstNullityMap();
    planeMap = shdDat->ConstPlaneMap();
    planes = *(shdDat->ConstPlanes());
    memcpy(objectMap, shdDat->ConstPlaneMap(), sizeof(cl_int)*IMAGE_WIDTH*IMAGE_HEIGHT);
    objects.clear();

    borderLines.clear();
    IdPairs.clear();
    virutalPixels.clear();
    mapIdIndex.clear();
    betweenAngles.clear();
    pointPairs.clear();
    borderPoints.clear();
    heights.clear();

    std::sort(planes.begin(), planes.end(), [](const Segment& a, const Segment& b)
                                              { return a.numpt > b.numpt; });
}

//--------------------------------------------------

void ObjectClusterer::AbsorbTinyPlanes()
{
    const int smallLimit = 100;
    for(auto ref=planes.begin(); ref+1!=planes.end(); ++ref)
    {
        if(ref->id==MERGED_PLANE)
            continue;
        for(auto cmp=planes.rbegin(); cmp->id!=ref->id; ++cmp)
        {
            if(cmp->id==MERGED_PLANE)
                continue;
            if(cmp->numpt > smallLimit)
                break;
            if(CanAbsorbSmallPlane(*ref, *cmp))
                AbsorbPlane(*ref, *cmp);
        }
    }
}

bool ObjectClusterer::CanAbsorbSmallPlane(const Segment& largerPlane, const Segment& smallerPlane)
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

    return IncludeTinyPlane(largerPlane, smallerPlane);
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

void ObjectClusterer::AbsorbPlane(Segment& largerPlane, Segment& smallerPlane)
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

//--------------------------------------------------

void ObjectClusterer::MergeLargePlanes()
{
    vecSegment tempPlanes;
    for(size_t i=0; i<planes.size(); i++)
        if(planes[i].id!=MERGED_PLANE)
            tempPlanes.push_back(planes[i]);
    planes.swap(tempPlanes);

    vecPairOfInts mergeList;
    for(size_t i=0; i<planes.size()-1; i++)
    {
        for(size_t k=i+1; k<planes.size(); k++)
        {
            if(ArePlanesInTheSameObject(planes[i], planes[k]))
                mergeList.emplace_back(i, k);
        }
    }

    MergePlanePairs(planes, mergeList);
}

void ObjectClusterer::MergePlanePairs(vecSegment& planes, vecPairOfInts& mergeList)
{
    mapPairOfInts indexMap;
    for(size_t i=0; i<planes.size(); i++)
        indexMap[i] = i;

    int conqueror, merged;
    for(auto& indexPair : mergeList)
    {
        assert(indexMap.find(indexPair.first) != indexMap.end());
        assert(indexMap.find(indexPair.second) != indexMap.end());
        conqueror = indexMap[indexPair.first];
        merged = indexMap[indexPair.second];
        assert(planes[conqueror].id != MERGED_PLANE);
        assert(planes[merged].id != MERGED_PLANE);
//        qDebug() << "merge:" << indexPair.first << "<=" << indexPair.second << "become" << conqueror << "<=" << merged
//                    << "ID" << planes[conqueror].id << "<=" << planes[merged].id;
        if(planes[conqueror].id == planes[merged].id)
            continue;

        AbsorbPlane(planes[conqueror], planes[merged]);
        UpdateIndexMap(indexMap, merged, conqueror);    // value: merged->conqueror
    }
}

void ObjectClusterer::UpdateIndexMap(mapPairOfInts& indexMap, const int fromValue, const int becomeValue)
{
    for(auto& pair : indexMap)
    {
        if(pair.second==fromValue)
            pair.second = becomeValue;
    }
}

bool ObjectClusterer::ArePlanesInTheSameObject(const Segment& firstPlane, const Segment& secondPlane)
{
    dbgFirstId = firstPlane.id;
    dbgSecondId = secondPlane.id;
    if(DoRectsOverlap(firstPlane.rect, secondPlane.rect)==false)
        return false;
    ImRect ovlRect = OverlappingRect(firstPlane.rect, secondPlane.rect);

    vecPairOfPixels connPixels;
    if(ArePlanesConnected(ovlRect, firstPlane, secondPlane, connPixels)==false)
        return false;

    if(clAngleBetweenVectorsLessThan(firstPlane.normal, secondPlane.normal, 10.f, true))
        return true;

    const float angleDegree = InnerAngleBetweenPlanes(firstPlane, secondPlane, connPixels);
//    DetermineConvexityByHeight(firstPlane, secondPlane);
//    return false;

    if(angleDegree > 180.f) // concave
        return true;
    return !DetermineConvexity(firstPlane, secondPlane, angleDegree);
}

float ObjectClusterer::InnerAngleBetweenPlanes(const Segment& firstPlane, const Segment& secondPlane, const vecPairOfPixels& connPixels)
{
    ImLine border = FitLine2D(connPixels);
    IdPairs.emplace_back(firstPlane.id, secondPlane.id);
    borderLines.push_back(border);

    bool firstPlaneUp = IsFirstUpperPlane(border, connPixels);
    cl_int2 borderCenter = PickBorderCenter(border, firstPlane.id, secondPlane.id);
    const cl_float4& pointOnBorder = pointCloud[PIXIDX(borderCenter)];
    cl_float4 borderDirection = clCross(firstPlane.normal, secondPlane.normal);

    Segment scFirstPlane = ScalePlaneToIncludePoint(firstPlane, pointOnBorder);
    Segment scSecondPlane = ScalePlaneToIncludePoint(secondPlane, pointOnBorder);

    cl_float4 pointOnFirst = VirtualPointOnPlaneAroundBorder(scFirstPlane, border, borderCenter, firstPlaneUp);
    cl_float4 pointOnSecond = VirtualPointOnPlaneAroundBorder(scSecondPlane, border, borderCenter, !firstPlaneUp);
    pointPairs.emplace_back(pointOnFirst, pointOnSecond);
    borderPoints.push_back(pointOnBorder);

    cl_float4 firstDirFromBorder = PlaneDirectionFromBorder(scFirstPlane.normal, borderDirection, pointOnFirst - pointOnBorder);
    cl_float4 secondDirFromBorder = PlaneDirectionFromBorder(scSecondPlane.normal, borderDirection, pointOnSecond - pointOnBorder);
    float angleDegree = AngleBetweenVectorsDegree(firstDirFromBorder, secondDirFromBorder);
    betweenAngles.push_back(angleDegree);
    return angleDegree;
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
//        line.endPixels[0] = (cl_int2){range.xl, (int)(s(0)*range.xl + s(1))};
//        line.endPixels[1] = (cl_int2){range.xh, (int)(s(0)*range.xh + s(1))};
        line.endPixels[0] = (cl_int2){range.xl, line.GetY(range.xl)};
        line.endPixels[1] = (cl_int2){range.xh, line.GetY(range.xh)};
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
//        line.endPixels[0] = (cl_int2){(int)(s(0)*range.yl + s(1)), range.yl};
//        line.endPixels[1] = (cl_int2){(int)(s(0)*range.yh + s(1)), range.yh};
        line.endPixels[0] = (cl_int2){line.GetX(range.yl), range.yl};
        line.endPixels[1] = (cl_int2){line.GetX(range.yh), range.yh};
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

cl_int2 ObjectClusterer::PickBorderCenter(const ImLine& border, const int firstID, const int secondID)
{
    const cl_int2 centerPixel = (border.endPixels[0] + border.endPixels[1])/2;
    const int pxidx = PIXIDX(centerPixel);
    if(nullityMap[pxidx] < NullID::PointNull && (objectMap[pxidx]==firstID || objectMap[pxidx]==secondID))
        return centerPixel;
    else
        return SearchValidPixelOnLine(border, centerPixel, firstID, secondID);
}

cl_int2 ObjectClusterer::SearchValidPixelOnLine(const ImLine& border, const cl_int2& centerPixel, const int firstID, const int secondID)
{
    int majorAxis=0, minorAxis=1;
    if(border.IsXAxisMajor()==false)
    {
        majorAxis=1;
        minorAxis=0;
    }
    cl_int2 linePixel = centerPixel;
    int pxidx, move=1;
    int rangeMin = smin(border.endPixels[0].s[majorAxis], border.endPixels[1].s[majorAxis]);
    int rangeMax = smax(border.endPixels[0].s[majorAxis], border.endPixels[1].s[majorAxis]);
    cl_float4 coef = (cl_float4){border.a, border.b, border.c, 0.f};

    while(linePixel.s[majorAxis] >= rangeMin && linePixel.s[majorAxis] <= rangeMax)
    {
        linePixel.s[majorAxis] = centerPixel.s[majorAxis]-move;
        linePixel.s[minorAxis] = (int)(-coef.s[majorAxis]/coef.s[minorAxis]*linePixel.s[majorAxis] + coef.s[2]/coef.s[minorAxis]);
        pxidx = PIXIDX(linePixel);
        if(nullityMap[pxidx] < NullID::PointNull && (objectMap[pxidx]==firstID || objectMap[pxidx]==secondID))
        {
            qDebug() << "   Null Center" << majorAxis << centerPixel << move << linePixel << dbgFirstId << dbgSecondId
                     << pointCloud[PIXIDX(centerPixel)] << pointCloud[pxidx];
            return linePixel;
        }

        linePixel.s[majorAxis] = centerPixel.s[majorAxis]+move;
        linePixel.s[minorAxis] = (int)(-coef.s[majorAxis]/coef.s[minorAxis]*linePixel.s[majorAxis] + coef.s[2]/coef.s[minorAxis]);
        pxidx = PIXIDX(linePixel);
        if(nullityMap[pxidx] < NullID::PointNull && (objectMap[pxidx]==firstID || objectMap[pxidx]==secondID))
        {
            qDebug() << "   Null Center" << majorAxis << centerPixel << move << linePixel << dbgFirstId << dbgSecondId
                     << pointCloud[PIXIDX(centerPixel)]<< pointCloud[pxidx];
            return linePixel;
        }
        move++;
    }
    assert(0);
    return centerPixel;
}

Segment ObjectClusterer::ScalePlaneToIncludePoint(const Segment& srcPlane, const cl_float4& point)
{
    Segment dstPlane = srcPlane;
    const float normalDistBorder = fabsf(clDot(point, srcPlane.normal));
    const float normalDistPlane = fabsf(clDot(srcPlane.center, srcPlane.normal));
    dstPlane.center = srcPlane.center/normalDistPlane*normalDistBorder;
    return dstPlane;
}

cl_float4 ObjectClusterer::VirtualPointOnPlaneAroundBorder(const Segment& plane, const ImLine& border, const cl_int2& borderCenter, bool upperPlane)
{
    const int pixelDist = 10;
    cl_int2 virtualPixel;
    ImLine orthLine;
    orthLine.OrthogonalTo(border, borderCenter);
    if(orthLine.IsXAxisMajor())
    {
        if(upperPlane)
            virtualPixel.x = borderCenter.x + pixelDist;
        else
            virtualPixel.x = borderCenter.x - pixelDist;
        virtualPixel.y = orthLine.GetY(virtualPixel.x);
    }
    else
    {
        if(upperPlane)
            virtualPixel.y = borderCenter.y + pixelDist;
        else
            virtualPixel.y = borderCenter.y - pixelDist;
        virtualPixel.x = orthLine.GetX(virtualPixel.y);
    }
    virutalPixels.push_back(virtualPixel);

    const float normalDistBorder = fabsf(clDot(pointCloud[PIXIDX(borderCenter)], plane.normal));
    const cl_float4 rayDir = ImageConverter::ConvertPixelToPoint(virtualPixel.x, virtualPixel.y, 1.f);
    cl_float4 pointOnPlane = rayDir/fabsf(clDot(rayDir, plane.normal))*normalDistBorder;
        assert(fabsf(normalDistBorder - fabsf(clDot(pointOnPlane, plane.normal))) < 0.001f);
    return pointOnPlane;
}

cl_float4 ObjectClusterer::PlaneDirectionFromBorder(const cl_float4& thisPlaneNormal, const cl_float4& borderDirection, const cl_float4& roughDirection)
{
    cl_float4 planeDirection = clCross(thisPlaneNormal, borderDirection);
    if(clDot(planeDirection, roughDirection) < 0.f)
        planeDirection = planeDirection*(-1.f);
    return clNormalize(planeDirection);
}

float ObjectClusterer::AngleBetweenVectorsDegree(const cl_float4& v1, const cl_float4& v2)
{
    cl_float4 eye = (cl_float4){1,0,0,0};
    bool concave = (clDot(eye,v1) + clDot(eye,v2) > 0);
    float angleDegree = RAD2DEG(acosf(clDot(v1,v2)));
    if(concave)
        return 360.f - angleDegree;
    else
        return angleDegree;
}

bool ObjectClusterer::DetermineConvexity(const Segment& firstPlane, const Segment& secondPlane, const float angleDegree)
{    
    float relativeHeight;
    if(firstPlane.numpt > secondPlane.numpt)
        relativeHeight = HeightFromPlane(secondPlane, firstPlane);
    else
        relativeHeight = HeightFromPlane(firstPlane, secondPlane);
    const float convexityHeight = smax(DEPTH(secondPlane.center)*DEPTH(secondPlane.center)*0.01f, 0.007f);
    heights.emplace_back(relativeHeight, convexityHeight);

    if(angleDegree > 160.f && relativeHeight > convexityHeight)
        return true;
    else if(relativeHeight > convexityHeight*2.f)
        return true;
    return false;
}

float ObjectClusterer::HeightFromPlane(const Segment& inputPlane, const Segment& basePlane)
{
    const float baseDist = fabsf(clDot(basePlane.center, basePlane.normal));
    float minDist = 1000.f;
    int yitv = smax((inputPlane.rect.yh - inputPlane.rect.yl)/10, 1);
    int xitv = smax((inputPlane.rect.xh - inputPlane.rect.xl)/10, 1);
    int pxidx;

    for(int y=inputPlane.rect.yl; y<=inputPlane.rect.yh; y+=yitv)
    {
        for(int x=inputPlane.rect.xl; x<=inputPlane.rect.xh; x+=xitv)
        {
            pxidx = IMGIDX(y,x);
            if(nullityMap[pxidx] < NullID::PointNull && objectMap[pxidx]==inputPlane.id)
                minDist = smin(minDist, fabsf(clDot(pointCloud[pxidx], basePlane.normal)));
        }
    }
    return baseDist - minDist;
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

