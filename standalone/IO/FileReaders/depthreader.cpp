#include "depthreader.h"

DepthReader::DepthReader(const QString datapath)
    : depthScale(1)
    , indexScale(1)
{
    dsroot = QString("/home/cideep/Work/datatset");
    datasetPath = dsroot + datapath;
    ListRgbdInDir(datasetPath);
    CreateListFile(datasetPath);
    indexScale = smax(depthList.size()/500, 1);
    if(datapath.contains("CoRBS"))
        depthScale = 5;
    qDebug() << "DepthReader loaded" << depthList.size() << "rgbd files, depthScale:" << depthScale << ", indexScale:" << indexScale;
}

void DepthReader::ListRgbdInDir(const QString datapath)
{
    QString depthDir = datapath + "/depth";
    QDir dir(depthDir);
    QStringList filter;
    if(datapath.contains("rgbd-scenes"))
        filter << "*_depth.png";
    else
        filter << "*.png";
    depthList = dir.entryList(filter, QDir::Files, QDir::Name);
    if(depthList.empty())
        throw TryFrameException(QString("ListRgbdInDir: no image file in ") + depthDir);
    for(auto& filename : depthList)
        filename = depthDir + QString("/") + filename;
    qDebug() << depthList.size() << "rgbd files loaded";
}

void DepthReader::CreateListFile(const QString datapath)
{
    int radius = (int)(DESC_RADIUS*100.f);
    QString dstPath = datapath + QString("/DescriptorR%1").arg(radius);
    QDir dir;
    if(!dir.exists(dstPath))
        if(!dir.mkpath(dstPath))
            throw TryFrameException("failed to create directory");

    QString filename = dstPath + QString("/depthList.txt");
    depthListFile.setFileName(filename);
    if(depthListFile.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw TryFrameException(QString("cannot create depth list file ")+filename);
    qDebug() << "depthListFile" << filename;
}

void DepthReader::ReadRgbdPose(const int index, QImage& rgbimg, QImage& depimg, Pose6dof& pose)
{
    static QImage colorImg(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static QImage depthImg(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static int index_before=-1;
    int frameIndex = index*indexScale;

    if(frameIndex >= depthList.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(frameIndex).arg(depthList.size()));

    if(index_before == frameIndex)
    {
        rgbimg = colorImg;
        depimg = depthImg;
        return;
    }
    else
        index_before = frameIndex;

    QTextStream writer(&depthListFile);
    writer << depthList.at(frameIndex) << "\n";

    // set color as a gray image
    colorImg.fill(qRgb(200,200,200));

    // set depth image
    qDebug() << "depth reader read:" << depthList.at(frameIndex);
    cv::Mat rawImage = cv::imread(depthList.at(frameIndex).toUtf8().data(), cv::IMREAD_ANYDEPTH);
    if(rawImage.rows==0 || rawImage.type()!=CV_16U)
        throw TryFrameException(QString("depth image is not valid: ") + depthList.at(frameIndex));

    uint depth;
    QRgb dpethInRgb;
    const int scale = rawImage.rows / IMAGE_HEIGHT;
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            // read depth
            if(scale==1)
                depth = (uint)rawImage.at<ushort>(y*scale, x*scale);
            else
            {
                uint vcnt=0;
                depth = 0;
                if((uint)rawImage.at<ushort>(y*scale, x*scale) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<ushort>(y*scale, x*scale);
                if((uint)rawImage.at<ushort>(y*scale+1, x*scale) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<ushort>(y*scale+1, x*scale);
                if((uint)rawImage.at<ushort>(y*scale, x*scale+1) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<ushort>(y*scale, x*scale+1);
                if((uint)rawImage.at<ushort>(y*scale+1, x*scale+1) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<ushort>(y*scale+1, x*scale+1);

                if(vcnt>0)
                    depth = (uint)((float)depth/(float)vcnt);
            }

            depth /= depthScale;
            dpethInRgb = qRgb(0, (depth>>8 & 0xff), (depth & 0xff));
            depthImg.setPixel(x, y, dpethInRgb);
        }
    }

    rgbimg = colorImg;
    depimg = depthImg;
}
