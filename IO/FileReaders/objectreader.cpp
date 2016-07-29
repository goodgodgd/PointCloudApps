#include "objectreader.h"

QStringList ObjectReader::categoryDirs;
int ObjectReader::categoryIndex = 1;
int ObjectReader::instanceIndex = 1;
int ObjectReader::videoIndex = 1;
int ObjectReader::frameIndex = 0;

ObjectReader::ObjectReader()
{
    frameIndex = frameBegin-1;
    QDir dir(dsroot);
    categoryNames = dir.entryList(QDir::AllDirs, QDir::Name);
    categoryNames.removeFirst();
    categoryDirs = ListSubPaths(dsroot);
    instanceDirs = ListSubPaths(categoryDirs[categoryIndex]);

//    {
//        QDebug dbg = qDebug();
//        for(auto name: categoryNames)
//            dbg << name;
//    }
//    {
//        QDebug dbg = qDebug();
//        for(int i=0; i<5; i++)
//            dbg << categoryDirs[i];
//    }
}

QStringList ObjectReader::ListSubPaths(QString parentPath)
{
    QDir dir(parentPath);
    QStringList subDirs = dir.entryList(QDir::AllDirs, QDir::Name);
    subDirs.removeFirst();

    for(auto& dir : subDirs)
        dir = parentPath + QString("/") + dir;
    return subDirs;
}

void ObjectReader::ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose)
{
    UpdateIndices();
    ObjPointCloud::Ptr objPoints = ReadPointCloud(PCDName());
    ExtractRgbDepth(objPoints, color, depth);
}

void ObjectReader::UpdateIndices()
{
    frameIndex++;
    if(frameIndex <= frameEnd)
        return;
    frameIndex = frameBegin;

    videoIndex++;
    if(videoIndex <= videosUpto)
        return;
    videoIndex = 1;

    instanceIndex++;
    if(instanceIndex < instanceDirs.size())
        return;
    instanceIndex = 1;

    categoryIndex++;
    if(categoryIndex >= categoryDirs.size())
        throw TryFrameException("All the dataset is processed. Stop");

    instanceDirs = ListSubPaths(categoryDirs[categoryIndex]);
}

ObjPointCloud::Ptr ObjectReader::ReadPointCloud(QString fileName)
{
    static pcl::PointCloud<ObjPoint>::Ptr cloud (new pcl::PointCloud<ObjPoint>);

    if(pcl::io::loadPCDFile<ObjPoint>(fileName.toStdString(), *cloud) == -1) //* load the file
        throw TryFrameException(QString("pcd file does not exists: ")+fileName);
    qDebug() << "Loaded" << cloud->width << cloud->height  << "data points from" << fileName;

    return cloud;
}

QString ObjectReader::PCDName()
{
    return instanceDirs.at(instanceIndex) + QString("/") + categoryNames.at(categoryIndex)
            + QString("_%1_%2_%3.pcd").arg(instanceIndex).arg(videoIndex).arg(frameIndex);
}

void ObjectReader::ExtractRgbDepth(ObjPointCloud::Ptr pointCloud, QImage& colorImgOut, QImage& depthImgOut)
{
    static QImage colorImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static QImage depthImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    cl_uint depth;
    QRgb depthRgb, colorRgb;
    cl_uint2 pixel;
    FloatRGB frgb;
    colorImage.fill(qRgb(0,0,0));
    depthImage.fill(qRgb(0,0,0));

    for(size_t i=0; i<pointCloud->points.size(); i++)
    {
        pixel = (cl_uint2){pointCloud->points[i].imX - IMAGE_WIDTH/2, pointCloud->points[i].imY - IMAGE_HEIGHT/2};

        frgb.data = pointCloud->points[i].rgb;
        colorRgb = qRgb((int)frgb.rgb[0], (int)frgb.rgb[1], (int)frgb.rgb[2]);
        colorImage.setPixel(pixel.x, pixel.y, colorRgb);

        depth = (cl_uint)(pointCloud->points[i].y * 1000.f);
        depthRgb = qRgb(0, (depth>>8 & 0xff), (depth & 0xff));
        depthImage.setPixel(pixel.x, pixel.y, depthRgb);

        cl_float4 point = ImageConverter::PixelToPoint(pixel.x, pixel.y, pointCloud->points[i].y);

//        if(i%1000==100)
//            qDebug() << "objPoint" << pixel << pointCloud->points[i].y << pointCloud->points[i].x << pointCloud->points[i].z
//                         << "chkPoint" << point
//                            << "rgb" << frgb.rgb[0] << frgb.rgb[1] << frgb.rgb[2];
    }

    colorImgOut = colorImage;
    depthImgOut = depthImage;
}
