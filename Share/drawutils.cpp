#include "drawutils.h"

DrawUtils::DrawUtils()
{
}

void DrawUtils::DrawPointCloud(cl_float4* pointCloud, cl_float4* normalCloud, int viewOption, QImage colorImg, cl_float4* descriptorCloud)
{
    if(colorImg.width() != IMAGE_WIDTH || colorImg.height() != IMAGE_HEIGHT)
        return;
    if(viewOption == ViewOpt::ViewNone)
        return;

    // point color: white
    const cl_float4 nullgray = cl_float4{0.5f,0.5f,0.5f,0.5f};
    const float normalLength = 0.02f;
    cl_float4 ptcolor;
    int x, y;

    // add point cloud with size of 2
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        if(clIsNull(pointCloud[i]))
            continue;

        y = i/IMAGE_WIDTH;
        x = i%IMAGE_WIDTH;

        // set point color from color image
        if(clIsNull(normalCloud[i]))
            ptcolor = nullgray;
        else
            ptcolor = ConvertToColor(viewOption, colorImg.pixel(x, y), descriptorCloud[i]);

        // add point vertex
        gvm::AddVertex(VertexType::point, pointCloud[i], ptcolor, normalCloud[i], 3);

        // add line vertices
        if(viewOption & ViewOpt::Normal)
            if(y%NORMAL_INTERV==0 && x%NORMAL_INTERV==0)
                DrawNormal(pointCloud[i], normalCloud[i], ptcolor, normalLength);
    }
}

void DrawUtils::DrawNormalCloud(cl_float4* pointCloud, cl_float4* normalCloud, const cl_float4 ptcolor)
{
    // point color: white
    const float normalLength = 0.02f;

    // add point cloud with size of 2
    for(int y=0; y<IMAGE_HEIGHT; y+=NORMAL_INTERV)
    {
        for(int x=0; x<IMAGE_WIDTH; x+=NORMAL_INTERV)
        {
            int idx = IMGIDX(y,x);
            if(clIsNull(pointCloud[idx]) || clIsNull(normalCloud[idx]))
                continue;

            if(y%NORMAL_INTERV==0 && x%NORMAL_INTERV==0)
                DrawNormal(pointCloud[idx], normalCloud[idx], ptcolor, normalLength);
        }
    }
}

cl_float4 DrawUtils::ConvertToColor(int viewOption, QRgb rgb, cl_float4& descriptor)
{
    cl_float4 ptcolor;
    if(viewOption & ViewOpt::Color)
        ptcolor << rgb;
    else if(viewOption & ViewOpt::Descriptor)
        ptcolor = DescriptorToColor(descriptor);
    else
        ptcolor = (cl_float4){0.f,0.f,0.f,0.f};
    return ptcolor;
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

void DrawUtils::MarkPoint3D(cl_float4 point, cl_float4 normal, int viewOption, QRgb color, cl_float4 descriptor, const float normalLength)
{
    cl_float4 ptcolor = ConvertToColor(viewOption, color, descriptor);
    DrawNormal(point, normal, ptcolor, normalLength);
//    qDebug() << "picked descriptor" << descriptor;
}

void DrawUtils::DrawOnlyNeighbors(QPoint pixel, cl_float4* pointCloud, cl_float4* normalCloud
                                  , cl_int* neighborIndices, cl_int* numNeighbors
                                  , int viewOption, QImage& colorImg, cl_float4* descriptorCloud)
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
        ptcolor = ConvertToColor(viewOption, colorImg.pixel(pixel), descriptorCloud[ptidx]);
        gvm::AddVertex(VertexType::point, pointCloud[ptidx], ptcolor, normalCloud[ptidx], 2);
    }
}
