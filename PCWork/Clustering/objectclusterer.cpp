#include "objectclusterer.h"

int dbgFirstId = 0;
int dbgSecondId = 0;

ObjectClusterer::ObjectClusterer()
{
    qDebug() << "SmallPlaneMerger";
}

void ObjectClusterer::MergePlanes()
{
    InitDebugData();
    MergeLargePlanes();
}

void ObjectClusterer::InitDebugData()
{
    borderLines.clear();
    IdPairs.clear();
    virutalPixels.clear();
    mapIdIndex.clear();
    betweenAngles.clear();
    pointPairs.clear();
    borderPoints.clear();
    heights.clear();
}

void ObjectClusterer::MergeLargePlanes()
{
//    vecSegment tempPlanes;
//    for(size_t i=0; i<planes.size(); i++)
//        if(planes[i].id!=MERGED_PLANE)
//            tempPlanes.push_back(planes[i]);
//    planes.swap(tempPlanes);

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
    DetermineConvexity(firstPlane, secondPlane, angleDegree);
    return false;

    if(angleDegree > 180.f) // concave
        return true;
    return !DetermineConvexity(firstPlane, secondPlane, angleDegree);
}

//------------------------------

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

//------------------------------

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
