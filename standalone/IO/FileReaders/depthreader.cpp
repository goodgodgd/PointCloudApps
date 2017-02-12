#include "depthreader.h"

DepthReader::DepthReader(const QString datapath)
{
    ListRgbdInDir(datapath);
    if(datapath.contains("CoRBS"))
        depthScale = 5;
    qDebug() << "DepthReader depthScale" << depthScale;
}

void DepthReader::ListRgbdInDir(const QString datapath)
{
    QString depthDir = datapath + "/frames";
    QDir dir(depthDir);
    QStringList filter;
    filter << "*.png";
    depthList = dir.entryList(filter, QDir::Files, QDir::Name);
    if(depthList.empty())
        throw TryFrameException(QString("ListRgbdInDir: no image file in ") + depthDir);
    for(auto& filename : depthList)
        filename = depthDir + QString("/") + filename;
    qDebug() << depthList.size() << "rgbd files loaded";
}

void DepthReader::ReadRgbdPose(const int index, QImage& rgbimg, QImage& depimg, Pose6dof& pose)
{
    static QImage colorImg(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static QImage depthImg(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static int index_before=-1;

    if(index >= depthList.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(index).arg(depthList.size()));

    if(index_before == index)
    {
        rgbimg = colorImg;
        depimg = depthImg;
        return;
    }
    else
        index_before = index;

    // set color as a gray image
    colorImg.fill(qRgb(200,200,200));

    // set depth image
    cv::Mat rawImage = cv::imread(depthList.at(index).toUtf8().data(), cv::IMREAD_ANYDEPTH);
    if(rawImage.rows==0 || rawImage.type()!=CV_16U)
        throw TryFrameException("depth image is not valid");

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
