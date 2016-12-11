#include "rgbdposereader.h"

QString RgbdPoseReader::dsetPath;
int RgbdPoseReader::DSID = 0;

RgbdPoseReader::RgbdPoseReader(const int DSID_)
{
    DSID = DSID_;
}

void RgbdPoseReader::ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose)
{
    color = ReadColor(ColorName(index));
    depth = ReadDepth(DepthName(index));
    pose = ReadPose(index);
    DrawTrajectory(trajectory, index);
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

            if(DSID >= DSetID::TUM_freiburg1_desk || DSID <= DSetID::Corbs_human)
                depth /= 5;

            rgb = qRgb(0, (depth>>8 & 0xff), (depth & 0xff));
            image.setPixel(x, y, rgb);
        }
    }
    return image;
}

Pose6dof RgbdPoseReader::ReadPose(const int index)
{
    if(trajectory.size() <= index)
        throw TryFrameException(QString("pose index is out of size %1<=%2").arg((int)trajectory.size()).arg(index));
    return trajectory[index];
}

void RgbdPoseReader::DrawTrajectory(const std::vector<Pose6dof>& trajectory, const int fromIndex)
{
    cl_float4 vertex;
    cl_float4 color = (cl_float4){1,1,0,0};
    cl_float4 normal = (cl_float4){0,0,1,0};
    Pose6dof relPose;
    int drawUpto = (int)smin((size_t)(fromIndex+600), trajectory.size());
    for(int i=fromIndex; i<drawUpto; ++i)
    {
        relPose = trajectory[fromIndex] / trajectory[i];
        vertex = (cl_float4){relPose.x, relPose.y, relPose.z, 0};
        gvm::AddVertex(VertexType::line, vertex, color, normal, 2);

        relPose = trajectory[fromIndex] / trajectory[i+1];
        vertex = (cl_float4){relPose.x, relPose.y, relPose.z, 0};
        gvm::AddVertex(VertexType::line, vertex, color, normal, 2, true);
    }
}
