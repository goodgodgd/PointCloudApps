#ifndef RGBDSYNCREADER_H
#define RGBDSYNCREADER_H

#include "rgbdposereader.h"

class RgbdSyncReader : public RgbdPoseReader
{
public:
    RgbdSyncReader(const QString localPath);

private:
    void LoadInitInfo(const QString datapath);
    void WriteDepthListInText(const QString datapath);
    void ReadFramePose(const int index, Pose6dof& pose) {}
    QString ColorName(const int index);
    QString DepthName(const int index);

    QStringList depthList;
    QStringList colorList;
};

#endif // RGBDSYNCREADER_H
