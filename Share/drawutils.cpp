#include "drawutils.h"

DrawUtils::DrawUtils()
{
}

void DrawUtils::DrawPointCloud(int viewOption, cl_float4* pointCloud, cl_float4* normalCloud, QImage colorImg, cl_float4* descriptorCloud)
{
    if(colorImg.width() != IMAGE_WIDTH || colorImg.height() != IMAGE_HEIGHT)
        return;

    // point color: white
    cl_float4 ptcolor = cl_float4{1,1,1,1};
    const float normalLength = 0.02f;
    QRgb pixelColor;
    int x, y;

    // add point cloud with size of 2
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        if(clIsNull(pointCloud[i]) || clIsNull(normalCloud[i]))
            continue;

        y = i/IMAGE_WIDTH;
        x = i%IMAGE_WIDTH;

        // set point color from color image
        if(viewOption & ViewOpt::Color)
        {
            pixelColor = colorImg.pixel(x, y);
            ptcolor << pixelColor;
        }
        else if(viewOption & ViewOpt::Descriptor)
        {
            ptcolor = DescriptorToColor(descriptorCloud[i]);
        }

        // add point vertex
        gvm::AddVertex(VertexType::point, pointCloud[i], ptcolor, normalCloud[i], 3);

        // add line vertices
        if(viewOption & ViewOpt::Normal)
        {
            cl_float4 normalTip = pointCloud[i] + normalCloud[i] * normalLength;
            if(y%5==2 && x%5==2)
            {
                gvm::AddVertex(VertexType::line, pointCloud[i], ptcolor, normalCloud[i], 1);
                gvm::AddVertex(VertexType::line, normalTip, ptcolor, normalCloud[i], 1, true);
            }
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

void DrawUtils::MarkNeighborsOnImage(QImage& srcimg, QPoint point, cl_int* neighborIndices, cl_int* numNeighbors)
{
    int nbstart = IMGIDX(point.y(), point.x()) * NEIGHBORS_PER_POINT;
    int numneigh = numNeighbors[IMGIDX(point.y(), point.x())];
    if(numneigh < 0 || numneigh > NEIGHBORS_PER_POINT)
        return;

    srcimg.setPixel(point, qRgb(255,0,0));

    qDebug() << "# neighbors" << numneigh;
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

void DrawUtils::MarkPoint3D(int viewOption, cl_float4 point, cl_float4 normal, QRgb color, cl_float4 descriptor)
{
    const float normalLength = 0.2f;
    cl_float4 ptcolor = cl_float4{1,1,1,1};

    if(viewOption & ViewOpt::Color)
        ptcolor << color;
    else if(viewOption & ViewOpt::Descriptor)
        ptcolor = DescriptorToColor(descriptor);

    // add long line on mark point
    cl_float4 normalTip = point + normal * normalLength;
    gvm::AddVertex(VertexType::line, point, ptcolor, normal, 1);
    gvm::AddVertex(VertexType::line, normalTip, ptcolor, normal, 1, true);

    qDebug() << "picked descriptor" << descriptor;
}

void DrawUtils::DrawOnlyNeighbors(QPoint pixel, cl_float4* pointCloud, cl_float4* normalCloud
                                  , cl_int* neighborIndices, cl_int* numNeighbors, QImage& colorImg)
{
    const int nbstart = IMGIDX(pixel.y(), pixel.x()) * NEIGHBORS_PER_POINT;
    const int numneigh = numNeighbors[IMGIDX(pixel.y(), pixel.x())];
    if(numneigh < 0 || numneigh > NEIGHBORS_PER_POINT)
        return;

    cl_float4 ptcolor = cl_float4{1,1,1,1};
    int ptidx;
    QRgb rgb;
    for(int i=nbstart; i<nbstart+numneigh; i++)
    {
        rgb = colorImg.pixel(pixel);
        ptcolor << rgb;
        ptidx = neighborIndices[i];
        gvm::AddVertex(VertexType::point, pointCloud[ptidx], ptcolor, normalCloud[ptidx], 2);
    }
}
