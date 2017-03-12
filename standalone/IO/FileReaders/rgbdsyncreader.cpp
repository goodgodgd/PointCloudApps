#include "rgbdsyncreader.h"

RgbdSyncReader::RgbdSyncReader(const QString localPath)
    : RgbdPoseReader(localPath)
{
    LoadInitInfo(curDatasetPath);
    WriteDepthListInText(curDatasetPath);
    indexScale = smax(depthList.size()/500, 1);
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

void RgbdSyncReader::WriteDepthListInText(const QString datapath)
{
    int radius = (int)(DESC_RADIUS*100.f);
    QString dstPath = datapath + QString("/DescriptorR%1").arg(radius);
    QDir dir;
    if(!dir.exists(dstPath))
        if(!dir.mkpath(dstPath))
            throw TryFrameException("failed to create directory");

    QString filename = dstPath + QString("/depthList.txt");
    QFile depthListFile(filename);
    if(depthListFile.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw TryFrameException(QString("cannot create depth list file ")+filename);
    qDebug() << "depthListFile" << filename;

    QTextStream writer(&depthListFile);
    for(auto& filename : depthList)
        writer << filename << "\n";
}

QString RgbdSyncReader::ColorName(const int index)
{
    int frameIndex = index * indexScale;
    if(frameIndex >= colorList.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(frameIndex).arg(colorList.size()));
    qDebug() << "ColorName" << colorList[frameIndex];
    return colorList[frameIndex];
}

QString RgbdSyncReader::DepthName(const int index)
{
    int frameIndex = index * indexScale;
    if(frameIndex >= depthList.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(frameIndex).arg(depthList.size()));
    qDebug() << "DepthName" << depthList[frameIndex];
    return depthList[frameIndex];
}
