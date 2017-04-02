#ifndef RGBDSYNCREADER_H
#define RGBDSYNCREADER_H

#include "rgbdposereader.h"

class RgbdSyncReader : public RgbdPoseReader
{
public:
    RgbdSyncReader(const QString localPath);
    virtual int GetLength() { return depthList.size(); }

private:
    void LoadInitInfo();
    void ReadFramePose(const int index, Pose6dof& pose) {}
    QString ColorName(const int frameIndex);
    QString DepthName(const int frameIndex);

    QStringList depthList;
    QStringList colorList;
};

#endif // RGBDSYNCREADER_H
