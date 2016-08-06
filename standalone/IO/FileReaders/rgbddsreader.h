#ifndef RGBDFILERW_H
#define RGBDFILERW_H

#include <stdio.h>
#include <QString>
#include <QImage>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include "Share/project_common.h"
#include "Share/annotation.h"

using namespace std;

namespace DBID {
    enum Enum
    {
        DESK1,
        DESK2,
        DESK3,
        MEETING
    };
}

#define DBPATH  "/home/hyukdoo/Work/PointCloudApps/rgbd-scenes"
typedef ushort  DepthType;

class RgbdDSReader
{
public:
    RgbdDSReader();
    static QString ColorName(const int dbID, const int index);
    static QString DepthName(const int dbID, const int index);
    static QString AnnotName(const int dbID, const int index);

    static bool ReadImage(const int dbID, const int index, QImage& colorImg, QImage& depthImg);
    static bool ReadColorImage(QString name, QImage& image_out);
    static bool ReadDepthImage(QString name, QImage& image_out);
    static void WriteImage(const int dbID, const int index, QImage& colorImg, cv::Mat depthMat);
    static void ReadAnnotations(const int dbID, const int index, vecAnnot& annots);
};

#endif // RGBDFILERW_H
