#include "objectclusterer.h"

#ifdef RESERVE_DEBUG_INFO
int dbgFirstId = 0;
int dbgSecondId = 0;
#endif

ObjectClusterer::ObjectClusterer()
{
}

void ObjectClusterer::MergePlanes()
{
    InitDebugData();
    MergeLargePlanes();
}

void ObjectClusterer::InitDebugData()
{
#ifdef RESERVE_DEBUG_INFO
    borderLines.clear();
    IdPairs.clear();
    virutalPixels.clear();
    mapIdIndex.clear();
    betweenAngles.clear();
    pointPairs.clear();
    borderPoints.clear();
    heights.clear();
#endif
}

void ObjectClusterer::MergeLargePlanes()
{
    ConvexDeterminerType convexityDeterminer = std::bind(&ObjectClusterer::DetermineConvexity, this, std::placeholders::_1, std::placeholders::_2);
    MergeableGraph mergeGraph(planes, convexityDeterminer);

    for(size_t i=0; i<planes.size()-1; ++i)     // from largest to smallest
    {
        for(size_t k=i+1; k<planes.size(); ++k)
        {
            if(ArePlanesInTheSameObject(planes[i], planes[k]))
                mergeGraph.AddConnection(i, k);
        }
    }

    MergePlanesThroughTree(mergeGraph);
}

void ObjectClusterer::MergePlanesThroughTree(MergeableGraph& mergeGraph)
{
    for(int i=0; i<planes.size(); ++i)
    {
        vecInts mergeList = mergeGraph.ExtractMergeList(i);
        for(int index : mergeList)
            AbsorbPlane(planes[i], planes[index]);
    }
}

bool ObjectClusterer::ArePlanesInTheSameObject(const Segment& firstPlane, const Segment& secondPlane)
{
#ifdef RESERVE_DEBUG_INFO
    dbgFirstId = firstPlane.id;
    dbgSecondId = secondPlane.id;
#endif
    if(DoRectsOverlap(firstPlane.rect, secondPlane.rect)==false)
        return false;
    ImRect ovlRect = OverlappingRect(firstPlane.rect, secondPlane.rect);

    vecPairOfPixels connPixels;
    if(ArePlanesConnected(ovlRect, firstPlane, secondPlane, connPixels)==false)
        return false;

    if(clAngleBetweenVectorsLessThan(firstPlane.normal, secondPlane.normal, 5.f, true))
        return true;

    float angleDegree;
    try{
        angleDegree = InnerAngleBetweenPlanes(firstPlane, secondPlane, connPixels);
    } catch (int code) {
        qDebug() << "exception code" << code;
        return false;
    }

    if(IsConcave(angleDegree))
        return true;
    return !DetermineConvexity(firstPlane, secondPlane);
}

bool ObjectClusterer::IsConcave(float angleDegree)
{
    return (angleDegree > 180.f);
}

bool ObjectClusterer::DetermineConvexity(const Segment& firstPlane, const Segment& secondPlane)
{
    float relativeHeight;
    if(firstPlane.numpt > secondPlane.numpt*3)
        relativeHeight = HeightFromPlane(secondPlane, firstPlane);
    else if(secondPlane.numpt > firstPlane.numpt*3)
        relativeHeight = HeightFromPlane(firstPlane, secondPlane);
    else
    {
        float relativeHeight1to2 = HeightFromPlane(secondPlane, firstPlane);
        float relativeHeight2to1 = HeightFromPlane(firstPlane, secondPlane);
        relativeHeight = smax(relativeHeight1to2, relativeHeight2to1);
    }

    const float convexityHeight = smax(DEPTH(firstPlane.center)*DEPTH(secondPlane.center)*0.015f, 0.007f);
#ifdef RESERVE_DEBUG_INFO
    heights.back().first = relativeHeight;
    heights.back().second = convexityHeight;
#endif
    return (relativeHeight > convexityHeight);
}


//------------------------------

float ObjectClusterer::InnerAngleBetweenPlanes(const Segment& firstPlane, const Segment& secondPlane, const vecPairOfPixels& connPixels)
{
    ImLine border = FitLine2D(connPixels);

    bool firstPlaneUp = IsFirstUpperPlane(border, connPixels);
    cl_int2 borderCenter = PickBorderCenter(border, firstPlane.id, secondPlane.id);
    const cl_float4& pointOnBorder = pointCloud[PIXIDX(borderCenter)];
    cl_float4 borderDirection = clCross(firstPlane.normal, secondPlane.normal);

    Segment scFirstPlane = ScalePlaneToIncludePoint(firstPlane, pointOnBorder);
    Segment scSecondPlane = ScalePlaneToIncludePoint(secondPlane, pointOnBorder);

    cl_float4 pointOnFirst = VirtualPointOnPlaneAroundBorder(scFirstPlane, border, borderCenter, firstPlaneUp);
    cl_float4 pointOnSecond = VirtualPointOnPlaneAroundBorder(scSecondPlane, border, borderCenter, !firstPlaneUp);

    cl_float4 firstDirFromBorder = PlaneDirectionFromBorder(scFirstPlane.normal, borderDirection, pointOnFirst - pointOnBorder);
    cl_float4 secondDirFromBorder = PlaneDirectionFromBorder(scSecondPlane.normal, borderDirection, pointOnSecond - pointOnBorder);
    float angleDegree = AngleBetweenVectorsDegree(firstDirFromBorder, secondDirFromBorder);

#ifdef RESERVE_DEBUG_INFO
    IdPairs.emplace_back(firstPlane.id, secondPlane.id);
    pointPairs.emplace_back(pointOnFirst, pointOnSecond);
    borderPoints.push_back(pointOnBorder);
    borderLines.push_back(border);
    betweenAngles.push_back(angleDegree);
    heights.emplace_back(0,0);
#endif
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
            return linePixel;

        linePixel.s[majorAxis] = centerPixel.s[majorAxis]+move;
        linePixel.s[minorAxis] = (int)(-coef.s[majorAxis]/coef.s[minorAxis]*linePixel.s[majorAxis] + coef.s[2]/coef.s[minorAxis]);
        pxidx = PIXIDX(linePixel);
        if(nullityMap[pxidx] < NullID::PointNull && (objectMap[pxidx]==firstID || objectMap[pxidx]==secondID))
            return linePixel;
        move++;
    }
    throw 1;
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
    cl_int2 vpixel1st, vpixel2nd, virtualPixel;
    ImLine orthLine;
    orthLine.OrthogonalTo(border, borderCenter);
    if(orthLine.IsXAxisMajor())
    {
        vpixel1st.x = borderCenter.x + pixelDist;
        vpixel1st.y = orthLine.GetY(vpixel1st.x);
        vpixel2nd.x = borderCenter.x - pixelDist;
        vpixel2nd.y = orthLine.GetY(vpixel2nd.x);
    }
    else
    {
        vpixel1st.y = borderCenter.y + pixelDist;
        vpixel1st.x = orthLine.GetX(vpixel1st.y);
        vpixel2nd.y = borderCenter.y - pixelDist;
        vpixel2nd.x = orthLine.GetX(vpixel2nd.y);
    }

    if(upperPlane)
        virtualPixel = (border.IsAboveLine(vpixel1st)) ? vpixel1st : vpixel2nd;
    else
        virtualPixel = (border.IsAboveLine(vpixel1st)) ? vpixel2nd : vpixel1st;
#ifdef RESERVE_DEBUG_INFO
    virutalPixels.push_back(virtualPixel);
#endif

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
