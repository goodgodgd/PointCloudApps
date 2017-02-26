#ifndef DEPTHREADER_H
#define DEPTHREADER_H

#include <QDir>
#include <QDate>
#include <QTime>
#include <QStringList>
#include <QImage>
#include <opencv2/opencv.hpp>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "IO/FileReaders/rgbdreaderinterface.h"

class DepthReader : public RgbdReaderInterface
{
public:
    DepthReader(const QString datapath);
    virtual void ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose);

protected:
    void ListRgbdInDir(const QString datapath);
    void CreateListFile(const QString datapath);
    QStringList depthList;
    int depthScale;
    int indexScale;
    QFile depthListFile;
};

#endif // DEPTHREADER_H
