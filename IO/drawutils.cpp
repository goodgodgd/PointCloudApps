#include "drawutils.h"

QImage DrawUtils::colorMap = QImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);

DrawUtils::DrawUtils()
{
}

void DrawUtils::DrawPointCloud(cl_float4* pointCloud, cl_float4* normalCloud)
{
    const cl_float4 nullgray = cl_float4{0.5f,0.5f,0.5f,0.5f};
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

void DrawUtils::DrawNormalCloud(cl_float4* pointCloud, cl_float4* normalCloud, const int normalInterval, const cl_float4 uniformColor)
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

cl_float4 DrawUtils::DescriptorToColor(cl_float4 descriptor)
{
    const float color_range = 14.f;
    cl_float4 color;
    color.x = descriptor.x / color_range + 0.5f;
    color.x = smin(smax(color.x, 0.f), 1.f);
    color.y = descriptor.y / color_range + 0.5f;
    color.y = smin(smax(color.y, 0.f), 1.f);
    color.z = (2.f - color.x - color.y) / 2.f;
    color.z = smin(smax(color.z, 0.f), 1.f);
    return color;
}

void DrawUtils::DrawNormal(const cl_float4& point, const cl_float4& normal, const cl_float4& ptcolor, const float length)
{
    cl_float4 normalTip = point + normal * length;
    gvm::AddVertex(VertexType::line, point, ptcolor, normal, 1);
    gvm::AddVertex(VertexType::line, normalTip, ptcolor, normal, 1, true);
}

void DrawUtils::MarkNeighborsOnImage(QImage& srcimg, QPoint point, cl_int* neighborIndices, cl_int* numNeighbors)
{
    int nbstart = IMGIDX(point.y(), point.x()) * NEIGHBORS_PER_POINT;
    int numneigh = numNeighbors[IMGIDX(point.y(), point.x())];
    if(numneigh < 0 || numneigh > NEIGHBORS_PER_POINT)
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

void DrawUtils::MarkPoint3D(cl_float4 point, cl_float4 normal, QRgb color, const float normalLength)
{
    cl_float4 ptcolor;
    ptcolor << color;
    DrawNormal(point, normal, ptcolor, normalLength);
//    qDebug() << "picked descriptor" << descriptor;
}

void DrawUtils::DrawOnlyNeighbors(QPoint pixel, cl_float4* pointCloud, cl_float4* normalCloud
                                  , cl_int* neighborIndices, cl_int* numNeighbors, QImage& colorImg)
{
    const int nbstart = IMGIDX(pixel.y(), pixel.x()) * NEIGHBORS_PER_POINT;
    const int numneigh = numNeighbors[IMGIDX(pixel.y(), pixel.x())];
    qDebug() << "# neighbors" << numneigh;
    if(numneigh < 0 || numneigh > NEIGHBORS_PER_POINT)
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

void DrawUtils::SetColorMapByRgbImage(const QImage& rgbImg)
{
    colorMap = rgbImg;
}

void DrawUtils::SetColorMapByDescriptor(const cl_float4* descriptors, const cl_uchar* nullityMap)
{
    const float color_range = 14.f;
    uchar r, g, b;
    int i;
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            i = IMGIDX(y,x);
            if(nullityMap[i]>=NullID::DescriptorNull)
                colorMap.setPixel(x, y, GetRandomColor(-2));
            else
            {
                r = (uchar)(smin(descriptors[i].x / color_range * 255.f, 255.f));
                g = (uchar)(smin(descriptors[i].y / color_range * 255.f, 255.f));
                b = (uchar)(255 - (r+g)/2);
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
        negaColors[2] = qRgb(100,100,100);
    }
    if(randColors.empty())
    {
        randColors.resize(10000);
        srand(0);
        for(auto& c : randColors)
            c = qRgb(rand()%255, rand()%255, rand()%255);
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






