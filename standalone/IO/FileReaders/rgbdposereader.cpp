#include "rgbdposereader.h"

RgbdPoseReader::RgbdPoseReader(const QString localPath)
    : depthScale(1)
    , indexScale(1)
{
    dataRootPath = QString(SCENEDATAROOT);
    curDatasetPath = dataRootPath + localPath;
}

void RgbdPoseReader::ReadRgbdFrame(const int index, QImage& color, QImage& depth)
{
    color = ReadColor(ColorName(index));
    depth = ReadDepth(DepthName(index));
}

QImage RgbdPoseReader::ReadColor(const QString name)
{
    static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    QImage rawImage(name);
    if(rawImage.isNull())
        throw TryFrameException(QString("dataset finished:") + name);
    image = rawImage.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    return image;
}

QImage RgbdPoseReader::ReadDepth(const QString name)
{
    static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    qDebug() << "depth name" << name;

    cv::Mat rawImage = cv::imread(name.toUtf8().data(), cv::IMREAD_ANYDEPTH);
    if(rawImage.rows==0 || rawImage.type()!=CV_16U)
        throw TryFrameException("depth image is not valid");

    uint depth;
    QRgb rgb;
    const int scale = rawImage.rows / IMAGE_HEIGHT;

    // convert depthMat to depthImg
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            // read depth
            if(scale==1)
                depth = (uint)rawImage.at<DepthType>(y*scale, x*scale);
            else
            {
                uint vcnt=0;
                depth = 0;
                if((uint)rawImage.at<DepthType>(y*scale, x*scale) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<DepthType>(y*scale, x*scale);
                if((uint)rawImage.at<DepthType>(y*scale+1, x*scale) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<DepthType>(y*scale+1, x*scale);
                if((uint)rawImage.at<DepthType>(y*scale, x*scale+1) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<DepthType>(y*scale, x*scale+1);
                if((uint)rawImage.at<DepthType>(y*scale+1, x*scale+1) > 0 && ++vcnt>=0)
                    depth += (uint)rawImage.at<DepthType>(y*scale+1, x*scale+1);

                if(vcnt>0)
                    depth = (uint)((float)depth/(float)vcnt);
            }

            depth /= depthScale;
            rgb = qRgb(0, (depth>>8 & 0xff), (depth & 0xff));
            image.setPixel(x, y, rgb);
        }
    }
    return image;
}
