#include "rgbdposereader.h"

QString RgbdPoseReader::dsetPath;
int RgbdPoseReader::DSID = 0;

RgbdPoseReader::RgbdPoseReader(const int DSID_)
{
    DSID = DSID_;
    qDebug() << "RgbdPoseReader constructor";
}

void RgbdPoseReader::ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose)
{
    color = ReadColor(ColorName(index));
    depth = ReadDepth(DepthName(index));
    pose = ReadPose(index);

//    static Pose6dof initPose;
//    if(g_frameIdx==0)
//        initPose = curPose;
//    pose = curPose / initPose;

//    qDebug() << "read image" << index << ColorName(index) << DepthName(index);
//    qDebugPrec(3) << "read pose" << pose << curPose << initPose;
}

QImage RgbdPoseReader::ReadColor(const QString name)
{
    static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    QImage rawImage(name);
    if(rawImage.isNull())
        throw TryFrameException(QString("color image is not valid:") + name);
    image = rawImage.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    return image;
}

QImage RgbdPoseReader::ReadDepth(const QString name)
{
    static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static cv::Mat resImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16U);

    cv::Mat rawImage = cv::imread(name.toUtf8().data(), cv::IMREAD_ANYDEPTH);
    if(rawImage.rows==0 || rawImage.type()!=CV_16U)
        throw TryFrameException("depth image is not valid");

    cv::resize(rawImage, resImage, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 0, 0, cv::INTER_NEAREST);

    uint depth;
    QRgb rgb;

    // convert depthMat to depthImg
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            // read depth
            depth = (uint)resImage.at<DepthType>(y,x);
            if(DSID >= DSetID::TUM_freiburg1_desk)
                depth /= 5;
//            if(x%100==50 && y%100==50)
//                qDebug() << "convert depth" << x << y << depth;
            rgb = qRgb(0, (depth>>8 & 0xff), (depth & 0xff));
            image.setPixel(x, y, rgb);
        }
    }
    return image;
}
