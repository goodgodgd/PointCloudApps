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
    qDebug() << "sample output file name" << fileName;
    QFile sampleLog(fileName);
    if(sampleLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException(QString("cannot open sample list file: ") + fileName);
    QTextStream reader(&sampleLog);
    int count=0;

    depthList.clear();
    pixels.clear();
    points.clear();
    while(!reader.atEnd())
    {
        QString line = reader.readLine();
        if(line.startsWith("#"))
            continue;

        QStringList fields = line.split(" ");
        if(fields.size() != 6)
            throw TryFrameException("invalid sample line");

        depthList.push_back(fields.at(0));
#ifdef SCALE_VAR
        pixels.push_back((cl_int2){fields.at(1).toInt()*2/SCALE_VAR, fields.at(2).toInt()*2/SCALE_VAR});
#else
        pixels.push_back((cl_int2){fields.at(1).toInt(), fields.at(2).toInt()});
#endif
        points.push_back((cl_float4){fields.at(3).toFloat(), fields.at(4).toFloat(), fields.at(5).toFloat(), 0});
        if(count<5)
            qDebug() << "first sample" << pixels.back() << points.back();
        count++;
    }
    qDebug() << "sample were loaded:" << depthList.size() << depthList.back();
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
#ifdef SCALE_VAR
    QRgb depthInRgb = depth.pixel(pixels[index].x, pixels[index].y);
    float depthRead = (float)(depthInRgb&0xffff) / 1000.f;
    qDebug() << "depth at sample pixel" << pixels[index] << depthRead << points[index].x;
    if(fabsf(depthRead - points[index].x) > 0.02f)
        qDebug() << "                   wrong depth at" << pixels[index] << pixels[index] << DepthName(index);
#endif
    color = colorImg;
}

std::vector<int> SampleReader::GetSamplePixel(const int index)
{
    if(index >= pixels.size())
        throw TryFrameException("sample pixel index exceeds pixels.size()");
    return {pixels[index].x, pixels[index].y};
}

QString SampleReader::ColorName(const int index)
{
    return QString("");
}

QString SampleReader::DepthName(const int index)
{
    qDebug() << "depth index" << index;
    if(index >= depthList.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(index).arg(depthList.size()));
    return depthList.at(index);
}
