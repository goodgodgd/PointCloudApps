#include "rgbdsyncreader.h"

RgbdSyncReader::RgbdSyncReader(const QString localPath)
    : RgbdPoseReader(localPath)
{
    LoadInitInfo(curDatasetPath);
}

void RgbdSyncReader::LoadInitInfo(const QString datapath)
{
    QString depthDir = datapath + "/depth";
    QDir dir(depthDir);
    QStringList filter;
    filter << "*_depth.png";
    depthList = dir.entryList(filter, QDir::Files, QDir::Name);
    if(depthList.empty())
        throw TryFrameException(QString("ListRgbdInDir: no image file in ") + depthDir);

    colorList.clear();
    for(QString& depthname : depthList)
    {
        depthname = depthDir + QString("/") + depthname;
        QString colorname = depthname;
        colorname.replace("_depth.png", ".png");
        colorList.append(colorname);
    }
    qDebug() << "first names" << depthList.at(0) << colorList.at(0);
    qDebug() << depthList.size() << colorList.size() << "rgbd files loaded";
}

QString RgbdSyncReader::ColorName(const int frameIndex)
{
    if(frameIndex >= colorList.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(frameIndex).arg(colorList.size()));
    return colorList[frameIndex];
}

QString RgbdSyncReader::DepthName(const int frameIndex)
{
    if(frameIndex >= depthList.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(frameIndex).arg(depthList.size()));
    return depthList[frameIndex];
}
