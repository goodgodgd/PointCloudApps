#include "drawutils.h"

QImage DrawUtils::colorMap = QImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);

DrawUtils::DrawUtils()
{
}

// 1. move draw functions to DrawUtils
// 2. add segmentmap to shared data
// 3. remove class variables from shared data

void DrawUtils::DrawPointCloud(int viewOption, SharedData* shdDat)
{
    if(viewOption == ViewOpt::ViewNone)
        return;

    if(viewOption & ViewOpt::Color)
        SetColorMapByRgbImage(shdDat->ConstColorImage());
    else if(viewOption & ViewOpt::Descriptor || viewOption & ViewOpt::CURVATURE)
        SetColorMapByDescriptor(shdDat->ConstDescriptors());
    else if(viewOption & ViewOpt::Segment)
        SetColorMapByCluster(shdDat->ConstPlaneMap());
    else if(viewOption & ViewOpt::Object)
        SetColorMapByCluster(shdDat->ConstObjectMap());
    else
        return;

    DrawPointCloudImpl(shdDat->ConstPointCloud(), shdDat->ConstNormalCloud());
    if(viewOption & ViewOpt::Normal)
        DrawNormalCloud(shdDat->ConstPointCloud(), shdDat->ConstNormalCloud());
    if(viewOption & ViewOpt::Segment)
        MarkSegments(shdDat->ConstPlanes());
    if(viewOption & ViewOpt::Object)
        MarkSegments(shdDat->ConstObjects());
}

void DrawUtils::SetColorMapByRgbImage(const QImage& rgbImg)
{
    colorMap = rgbImg;
}

void DrawUtils::SetColorMapByDescriptor(const DescType* descriptors)
{
    const float curvRange = 1.5f;
    uchar r, g, b;
    int i;
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            i = IMGIDX(y,x);
            if(clIsNull(descriptors[i]))
                colorMap.setPixel(x, y, GetRandomColor(-2));
            else
            {
                r = (uchar)(smin(smax(descriptors[i].x, -curvRange), curvRange) / curvRange * 127.f + 128.f);
                g = (uchar)(smin(smax(descriptors[i].y, -curvRange), curvRange) / curvRange * 127.f + 128.f);
                b = (uchar)(smin(smax(descriptors[i].z, -curvRange), curvRange) / curvRange * 127.f + 128.f);;
                colorMap.setPixel(x, y, qRgb(r,g,b));
            }
        }
    }
}

void DrawUtils::SetColorMapByCluster(const int* segmentMap)
{
    for(int y=0; y<IMAGE_HEIGHT; y++)
        for(int x=0; x<IMAGE_WIDTH; x++)
            colorMap.setPixel(x, y, GetRandomColor(segmentMap[IMGIDX(y,x)]));
}

void DrawUtils::DrawPointCloudImpl(const cl_float4* pointCloud, const cl_float4* normalCloud)
{
    cl_float4 ptcolor;
    int i;

    // add point cloud with size of 2
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            i = IMGIDX(y,x);
            if(clIsNull(pointCloud[i]))
                continue;

            ptcolor << colorMap.pixel(x, y);

            // add point vertex
            gvm::AddVertex(VertexType::point, pointCloud[i], ptcolor, normalCloud[i], 3);
        }
    }
}

void DrawUtils::DrawNormalCloud(const cl_float4* pointCloud, const cl_float4* normalCloud, const int normalInterval, const cl_float4 uniformColor)
{
    // point color: white
    const float normalLength = 0.02f;
    int i;
    bool isUniformValid = (uniformColor.x+uniformColor.y+uniformColor.z > 0.001f);
    cl_float4 ptcolor;
    if(isUniformValid)
        ptcolor = uniformColor;

    // add point cloud with size of 2
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            i = IMGIDX(y,x);
            if(clIsNull(pointCloud[i]))
                continue;

            if(isUniformValid==false)
                ptcolor << colorMap.pixel(x,y);

            if(y%normalInterval==0 && x%normalInterval==0)
                DrawNormal(pointCloud[i], normalCloud[i], ptcolor, normalLength);
        }
    }
}

void DrawUtils::DrawNormal(const cl_float4& point, const cl_float4& normal, const cl_float4& ptcolor, const float length)
{
    cl_float4 normalTip = point + normal * length;
    gvm::AddVertex(VertexType::line, point, ptcolor, normal, 1);
    gvm::AddVertex(VertexType::line, normalTip, ptcolor, normal, 1, true);
}

void DrawUtils::MarkSegments(const vecSegment* segments, const int minPts, const float normalLength)
{
    cl_float4 ptcolor;
    for(const Segment& seg : *segments)
    {
        if(seg.numpt > minPts)
        {
            ptcolor << GetRandomColor(seg.id);
            DrawNormal(seg.center, seg.normal, ptcolor, normalLength);
        }
    }
}

void DrawUtils::MarkPoint3D(SharedData* shdDat, const QPoint pixel, const float normalLength)
{
    const int ptidx = IMGIDX(pixel.y(),pixel.x());
    cl_float4 ptcolor;
    ptcolor << shdDat->ConstColorImage().pixel(pixel);
    DrawNormal(shdDat->ConstPointCloud()[ptidx], shdDat->ConstNormalCloud()[ptidx], ptcolor, normalLength);
//    qDebug() << "picked descriptor" << descriptor;
}

void DrawUtils::MarkPoint3D(const cl_float4 point, const cl_float4 normal, QRgb color, const float normalLength)
{
    static const cl_float4 defaultNormal = (cl_float4){0,0,1,0};
    cl_float4 ptcolor;
    ptcolor << color;

//    static int count=0;
//    if(point.x > 1.8f && qRed(color)==255 && qBlue(color)==0)
//        qDebug() << "deep point" << point << ++count;

    if(clIsNull(normal))
        DrawNormal(point, defaultNormal, ptcolor, normalLength);
    else
        DrawNormal(point, normal, ptcolor, normalLength);
}

void DrawUtils::MarkNeighborsOnImage(QImage& srcimg, QPoint point, cl_int* neighborIndices, cl_int* numNeighbors)
{
    int nbstart = IMGIDX(point.y(), point.x()) * MAX_NEIGHBORS;
    int numneigh = numNeighbors[IMGIDX(point.y(), point.x())];
    if(numneigh < 0 || numneigh > MAX_NEIGHBORS)
        return;

    srcimg.setPixel(point, qRgb(255,0,0));

    int ptidx, x, y;
    for(int i=nbstart; i<nbstart+numneigh; i++)
    {
        ptidx = neighborIndices[i];
        y = ptidx / IMAGE_WIDTH;
        x = ptidx % IMAGE_WIDTH;
        srcimg.setPixel(x, y, qRgb(255,100,100));
    }
}

void DrawUtils::DrawOnlyNeighbors(const QPoint pixel, const cl_float4* pointCloud, const cl_float4* normalCloud
                                  , const cl_int* neighborIndices, const cl_int* numNeighbors, const QImage& colorImg)
{
    const int nbstart = IMGIDX(pixel.y(), pixel.x()) * MAX_NEIGHBORS;
    const int numneigh = numNeighbors[IMGIDX(pixel.y(), pixel.x())];
    qDebug() << "# neighbors" << numneigh;
    if(numneigh < 0 || numneigh > MAX_NEIGHBORS)
        return;

    cl_float4 ptcolor;;
    int ptidx;
    for(int i=nbstart; i<nbstart+numneigh; i++)
    {
        ptidx = neighborIndices[i];
        ptcolor << colorImg.pixel(pixel);
        gvm::AddVertex(VertexType::point, pointCloud[ptidx], ptcolor, normalCloud[ptidx], 2);
    }
}

QRgb DrawUtils::GetRandomColor(int index)
{
    static std::vector<QRgb> negaColors;
    static std::vector<QRgb> randColors;

    // initialize colors
    if(negaColors.empty())
    {
        negaColors.resize(3);
        negaColors[0] = qRgb(255,255,255);
        negaColors[1] = qRgb(200,200,200);
        negaColors[2] = qRgb(50,50,50);
    }
    if(randColors.empty())
    {
        randColors.resize(10000);
        srand(0);
        for(auto& c : randColors)
            c = qRgb(rand()%200+55, rand()%200+55, rand()%200+55);
        randColors[0] = qRgb(255,255,255);
        randColors[1] = qRgb(255,0,0);
        randColors[2] = qRgb(0,255,0);
        randColors[3] = qRgb(0,0,255);
    }

    if(index<0 && -index<negaColors.size())
        return negaColors[-index];
    else if(index>=0 && index<randColors.size())
        return randColors[index];
    else
        return qRgb(0,0,0);
}

const QImage& DrawUtils::GetColorMap()
{
    return colorMap;
}
