#include "IO/rgbdfilerw.h"

RgbdFileRW::RgbdFileRW()
{
}

QString RgbdFileRW::ColorName(eDBID dbID, const int index)
{
    QString name;
    if(dbID==eDBID::DESK1)
        name = QString(DBPATH) + QString("/desk/desk_1/desk_1_%1.png").arg(index);
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2/desk_2_%1.png").arg(index);
    else if(dbID==eDBID::DESK3)
        name = QString(DBPATH) + QString("/desk/desk_3/desk_3_%1.png").arg(index);
    return name;
}

QString RgbdFileRW::DepthName(eDBID dbID, const int index)
{
    QString name;
    if(dbID==eDBID::DESK1)
        name = QString(DBPATH) + QString("/desk/desk_1/desk_1_%1_depth.png").arg(index);
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2/desk_2_%1_depth.png").arg(index);
    else if(dbID==eDBID::DESK3)
        name = QString(DBPATH) + QString("/desk/desk_3/desk_3_%1_depth.png").arg(index);
    return name;
}

QString RgbdFileRW::AnnotName(eDBID dbID, const int index)
{
    QString name;
    if(dbID==eDBID::DESK1)
        name = QString(DBPATH) + QString("/desk/desk_1_annot/desk_1_%1.txt").arg(index);
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2_annot/desk_2_%1.txt").arg(index);
    else if(dbID==eDBID::DESK3)
        name = QString(DBPATH) + QString("/desk/desk_3_annot/desk_3_%1.txt").arg(index);
    return name;
}

bool RgbdFileRW::ReadImage(eDBID dbID, const int index, QImage& colorImg, QImage& depthImg)
{
    bool result;
    result = ReadColorImage(ColorName(dbID, index), colorImg);
    if(result==false)
        return false;

    result = ReadDepthImage(DepthName(dbID, index), depthImg);
    if(result==false)
        return false;

    return true;
}

bool RgbdFileRW::ReadColorImage(QString name, QImage& image_out)
{
    static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    QImage rawImage(name);
    if(rawImage.isNull())
        return false;
    image = rawImage.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);
    image_out = image;
    return true;
}

bool RgbdFileRW::ReadDepthImage(QString name, QImage& image_out)
{
    static QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    static cv::Mat resImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16U);

    cv::Mat rawImage = cv::imread(name.toUtf8().data(), cv::IMREAD_ANYDEPTH);
    if(rawImage.rows==0 || rawImage.type()!=CV_16U)
        return false;

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
            rgb = qRgb(0, (depth>>8 & 0xff), (depth & 0xff));
            image.setPixel(x, y, rgb);
        }
    }

    image_out = image;
    return true;
}

void RgbdFileRW::WriteImage(eDBID dbID, const int index, QImage& colorImg, cv::Mat depthMat)
{
}

void RgbdFileRW::ReadAnnotations(eDBID dbID, const int index, vector<Annotation>& annots)
{
    // TODO: fill in this function to read annotation info at frame of index

    QFile file(AnnotName(dbID, index));
    int xl, xh, yl, yh, instance;
    char name[20];
    file.open(QIODevice::ReadOnly|QIODevice::Text);
    QTextStream in(&file);

    QString line = in.readLine();
    int number = line.toInt();
    if(number)
    {
        for(int i=0; i<number; i++)
        {
            QString Annotation_data = in.readLine();
            QStringList tmpList = Annotation_data.split(",");
            sprintf(name,"%s",tmpList.value(0).toStdString().data());
            instance = tmpList.value(1).toInt();
            yl = tmpList.value(2).toInt();
            yh = tmpList.value(3).toInt();
            xl = tmpList.value(4).toInt();
            xh = tmpList.value(5).toInt();
            annots.emplace_back(name, instance, xl, xh, yl, yh);
        }
    }
    file.flush();
    in.flush();
    file.close();
    // ....

    

}
