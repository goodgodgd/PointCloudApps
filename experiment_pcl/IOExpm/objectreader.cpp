#include "objectreader.h"

int ObjectReader::categoryIndex=1;
int ObjectReader::instanceIndex=1;
int ObjectReader::videoIndex=1;
int ObjectReader::frameIndex=0;

ObjectReader::ObjectReader()
{
    frameIndex = 0;
    categoryNames = ListSubPaths(dsroot);
    qDebug() << "categoryNames" << categoryNames.size() << categoryNames.at(1) << categoryNames.at(2);
    instanceNames = ListSubPaths(GetCategoryPath());
    qDebug() << "instanceNames" << instanceNames.size() << instanceNames.at(1) << instanceNames.at(2);
    videoFrames = ListVideoFrames();
    qDebug() << "videoFrames" << videoFrames.size() << videoFrames.at(1).size() << videoFrames.at(1).at(1);
}

QStringList ObjectReader::ListSubPaths(QString parentPath)
{
    QDir dir(parentPath);
    QStringList subDirs = dir.entryList(QDir::AllDirs, QDir::Name);
//    qDebug() << "listsubpath" << parentPath << subDirs.size();

    for(int i=0; i<subDirs.size(); i++)
    {
        if(subDirs.at(i).startsWith(QChar('.')) || subDirs.at(i).startsWith(QChar('_')))
            subDirs.removeAt(i--);
    }
    subDirs.insert(0, "tmp");   // to start from index 1
    return subDirs;
}

QString ObjectReader::GetCategoryPath()
{
    return QString(dsroot) + QString("/") + categoryNames[categoryIndex];
}

QString ObjectReader::GetInstancePath()
{
    return GetCategoryPath() + QString("/") + instanceNames[instanceIndex];
}

std::vector<QStringList> ObjectReader::ListVideoFrames()
{
    int videoIdx=0;
    QStringList fileList;
    std::vector<QStringList> videoFrameList;
    videoFrameList.push_back(fileList); // pad empty list at(0)
    while(videoFrameList.size() < videosUpto && videoIdx < videosUpto+5)
    {
        videoIdx++;
        fileList = GetVideoFrameNames(videoIdx);
        if(fileList.size() < framesPerVideo)
            continue;
        videoFrameList.push_back(fileList);
    }
//    qDebug() << "videoFrame" << videoFrameList.size() << videoIdx;

    if(videoFrameList.size() < videosUpto)
        throw TryFrameException("insufficient videos");

    return videoFrameList;
}

QStringList ObjectReader::GetVideoFrameNames(const int videoIdx)
{
    QStringList filters;
    filters.push_back(instanceNames.at(instanceIndex) + QString("_%1*").arg(videoIdx));
    QDir dir(GetInstancePath());
    QStringList fileList = dir.entryList(filters, QDir::Files, QDir::Name);
    if(fileList.empty())
        return fileList;

    for(int i=0; i<fileList.size(); i++)
    {
        if(fileList.at(i).startsWith(QChar('.')) || fileList.at(i).startsWith(QChar('_')))
            fileList.removeAt(i--);
    }
    if(fileList.size()>2)
    {
        fileList.removeFirst();
        fileList.removeFirst();
    }
    fileList.insert(0, "tmp");   // to start from index 1
    return fileList;
}


void ObjectReader::ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose)
{
    UpdateIndices();
    ObjPointCloud::Ptr objPoints = ReadPointCloud(PcdFilePath());
    ExtractRgbDepth(objPoints, color, depth);
}

void ObjectReader::UpdateIndices()
{
    frameIndex++;
    if(frameIndex < framesPerVideo)
        return;
    frameIndex = 1;

    videoIndex++;
    qDebug() << "video change" << videoIndex;
    if(videoIndex < videoFrames.size())
        return;
    videoIndex = 1;

    instanceIndex++;
    qDebug() << "instance change" << instanceIndex;
    if(instanceIndex < instanceNames.size())
    {
        videoFrames = ListVideoFrames();
        return;
    }
    instanceIndex = 1;

    categoryIndex++;
    qDebug() << "category change" << categoryIndex;
    if(categoryIndex >= categoryNames.size())
        throw TryFrameException("All the dataset is processed. Stop");

    instanceNames = ListSubPaths(GetCategoryPath());
    videoFrames = ListVideoFrames();
}

ObjPointCloud::Ptr ObjectReader::ReadPointCloud(QString fileName)
{
    static pcl::PointCloud<ObjPoint>::Ptr cloud (new pcl::PointCloud<ObjPoint>);

    if(pcl::io::loadPCDFile<ObjPoint>(fileName.toStdString(), *cloud) == -1) //* load the file
        throw TryFrameException(QString("pcd file does not exists: ")+fileName);
    qDebug() << "Loaded" << cloud->width << cloud->height  << "data points from" << fileName;

    return cloud;
}

QString ObjectReader::PcdFilePath()
{
    return GetInstancePath() + QString("/") + videoFrames.at(videoIndex).at(frameIndex);
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
        if(OUTSIDEIMG(pixel.y,pixel.x))
            continue;

        frgb.data = pointCloud->points[i].rgb;
        colorRgb = qRgb((int)frgb.rgb[2], (int)frgb.rgb[1], (int)frgb.rgb[0]);
        colorImage.setPixel(pixel.x, pixel.y, colorRgb);

        depth = (cl_uint)(pointCloud->points[i].y * 1000.f);
        depthRgb = qRgb(0, (depth>>8 & 0xff), (depth & 0xff));
        depthImage.setPixel(pixel.x, pixel.y, depthRgb);
    }

    colorImgOut = colorImage;
    depthImgOut = depthImage;
}

void ObjectReader::ChangeInstance()
{
    frameIndex = 0;
    videoIndex = 1;

    instanceIndex++;
    qDebug() << "instance change" << instanceIndex;
    if(instanceIndex < instanceNames.size())
    {
        videoFrames = ListVideoFrames();
        return;
    }
    instanceIndex = 1;

    categoryIndex++;
    qDebug() << "category change" << categoryIndex;
    if(categoryIndex >= categoryNames.size())
        throw TryFrameException("All the dataset is processed. Stop");

    instanceNames = ListSubPaths(GetCategoryPath());
    videoFrames = ListVideoFrames();
}
