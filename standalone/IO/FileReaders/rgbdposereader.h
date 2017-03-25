#ifndef RGBDPOSEREADER_H
#define RGBDPOSEREADER_H

#include <stdio.h>
#include <map>
#include <QStringList>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <opencv2/opencv.hpp>
#include "Share/fordescriptor.h"
#include "IO/glvertexmanager.h"
#include "rgbdreaderinterface.h"


typedef ushort  DepthType;
typedef std::map<int, QString> Pathmap;

class RgbdPoseReader : public RgbdReaderInterface
{
public:
    RgbdPoseReader(const QString localPath);
    virtual ~RgbdPoseReader() {}
    virtual void ReadRgbdFrame(const int index, QImage& color, QImage& depth);
    virtual void ReadFramePose(const int index, Pose6dof& pose) {}
    virtual int GetLength() { return 0; }

protected:
    virtual void LoadInitInfo(const QString datapath) = 0;
    virtual QString ColorName(const int index) = 0;
    virtual QString DepthName(const int index) = 0;

    void CreateOutputPath();
    void InitReader(const QString datapath);
    QImage ReadColor(const QString name);
    QImage ReadDepth(const QString name);
    void WriteDepthList(const QString depthName);

    int depthScale;
    QFile depthListFile;
};

#endif // RGBDPOSEREADER_H
