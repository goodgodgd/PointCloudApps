#ifndef RGBDFILERW_H
#define RGBDFILERW_H

#include <stdio.h>
#include <vector>
#include <QString>
#include <QImage>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>
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

class RgbdFileRW
{
public:
    RgbdFileRW();
    static void ReadImage(QString folderpath, const int index, QImage& color, QImage& depth_rgb, QImage& depth_gray);
    static void WriteImage(QString folderpath, const int index, QImage& color, QImage& depth_rgb);
    static void ReadAnnotations(QString folderpath, const int index, vector<Annotation>& annots);
};

#endif // RGBDFILERW_H
