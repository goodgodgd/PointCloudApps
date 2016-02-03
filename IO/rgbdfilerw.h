#ifndef RGBDFILERW_H
#define RGBDFILERW_H

#include <stdio.h>
#include <vector>
#include <QString>
#include <QImage>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include "project_common.h"
using namespace std;

struct Annotation
{
    Annotation(const char* _name, int _instance, int _xl, int _xh, int _yl, int _yh)
    {
        sprintf(name, "%s", _name);
        instance = _instance;
        xl = _xl;
        xh = _xh;
        yl = _yl;
        yh = _yh;
    }

    char name[20];          // category name
    ushort instance, xl, xh, yl, yh;  // x, y bound box (low, high)
};

enum eDBID
{
    DESK1,
    DESK2,
    DESK3
};

#define DBPATH  "/home/hyukdoo/Work/rgbd-scenes"
typedef ushort  DepthType;

class RgbdFileRW
{
public:
    RgbdFileRW();
    static QString ColorName(eDBID dbID, const int index);
    static QString DepthName(eDBID dbID, const int index);
    static QString AnnotName(eDBID dbID, const int index);

    static void ReadImage(eDBID dbID, const int index, QImage& colorImg, cv::Mat& depthMat, QImage& depthImg);
    static void WriteImage(eDBID dbID, const int index, QImage& colorImg, cv::Mat depthMat);
    static void ReadAnnotations(eDBID dbID, const int index, vector<Annotation>& annots);
};

#endif // RGBDFILERW_H
