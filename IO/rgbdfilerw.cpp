#include "IO/rgbdfilerw.h"
//#include "png.h"
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
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2/desk_2_%1.png").arg(index);
    return name;
}

QString RgbdFileRW::DepthName(eDBID dbID, const int index)
{
    QString name;
    if(dbID==eDBID::DESK1)
        name = QString(DBPATH) + QString("/desk/desk_1/desk_1_%1_depth.png").arg(index);
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2/desk_2_%1_depth.png").arg(index);
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2/desk_2_%1_depth.png").arg(index);
    return name;
}

QString RgbdFileRW::AnnotName(eDBID dbID, const int index)
{
    QString name;
    if(dbID==eDBID::DESK1)
        name = QString(DBPATH) + QString("/desk/desk_1_annot/desk_1_%1.png").arg(index);
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2_annot/desk_2_%1.png").arg(index);
    else if(dbID==eDBID::DESK2)
        name = QString(DBPATH) + QString("/desk/desk_2_annot/desk_2_%1.png").arg(index);
    return name;
}

void RgbdFileRW::ReadImage(eDBID dbID, const int index, QImage& colorImg, cv::Mat& depthMat, QImage& depthImg)
{
    qDebug() << ColorName(dbID, index);
    qDebug() << DepthName(dbID, index);

    // load color image
    colorImg = QImage(ColorName(dbID, index));
    colorImg = colorImg.scaled(IMAGE_WIDTH, IMAGE_HEIGHT, Qt::KeepAspectRatio);

    // load depth image
    cv::Mat depthRaw = cv::imread(DepthName(dbID, index).toStdString(), CV_LOAD_IMAGE_ANYDEPTH);
    if(depthRaw.rows==0 || depthRaw.type()!=CV_16U)
        return;
    cv::resize(depthRaw, depthMat, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 0, 0, cv::INTER_NEAREST);
    qDebug() << "depthraw" << depthRaw.cols << depthRaw.rows << depthRaw.depth() << depthRaw.type();

    // init depthImg
    if(depthImg.width() != IMAGE_WIDTH || depthImg.height() != IMAGE_HEIGHT)
        depthImg = QImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);

    float depth;
    QRgb rgb;
    uchar gray;

    // convert depthMat to depthImg
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            // read depth
            depth = (float)depthMat.at<DepthType>(y,x);
            // set gray scale depth
            gray = (uchar)(depth / (float)DEPTH_RANGE * 255.f);
            rgb = qRgb(gray, gray, gray);
            depthImg.setPixel(x, y, rgb);

//            if(y%100==0 && x%100==0)
//                qDebug() << y << x << depthMat.at<DepthType>(y,x) << depth << gray << rgb;
        }
    }
}

void RgbdFileRW::WriteImage(eDBID dbID, const int index, QImage& colorImg, cv::Mat depthMat)
{
    cv::Mat mat(100, 100, CV_8U);
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
    if(number){
        for(int i=0; i<number; i++){
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
