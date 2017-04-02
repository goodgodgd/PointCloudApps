#include "samplereader.h"

SampleReader::SampleReader(const QString localPath)
    : RgbdPoseReader(localPath)
{
    LoadInitInfo();
    if(curDatasetPath.contains("CoRBS"))
        depthScale = 5;
}

void SampleReader::LoadInitInfo()
{
    // read sample list
    QString fileName = curOutputPath + QString("/sampleList.txt");
    QFile sampleLog(fileName);
    if(sampleLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException(QString("cannot open sample list file: ") + fileName);
    QTextStream reader(&sampleLog);
    int count=0;

    while(!reader.atEnd())
    {
        QString line = reader.readLine();
        if(line.startsWith("#"))
            continue;

        QStringList fields = line.split(" ");
        if(fields.size() != 3)
            throw TryFrameException("invalid sample line");

        depthList.push_back(fields.at(0));
        imgXList.push_back(fields.at(1).toInt());
        imgYList.push_back(fields.at(2).toInt());
        count++;
    }
    qDebug() << "sample were loaded:" << depthList.size() << imgXList.size() << imgYList.size();
}

void SampleReader::ReadRgbdFrame(const int index, QImage& color, QImage& depth)
{
    static QImage colorImg;
    if(colorImg.isNull())
    {
        colorImg = QImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
        colorImg.fill(qRgb(150,150,150));
    }

    depth = ReadDepth(DepthName(index));
    qDebug() << "depth" << depth.width() << depth.height();
    color = colorImg;
}

std::vector<int> SampleReader::GetSamplePixel(const int index)
{
    return {imgXList[index], imgYList[index]};
}

QString SampleReader::ColorName(const int index)
{
    return QString("");
}

QString SampleReader::DepthName(const int index)
{
    return depthList.at(index);
}
