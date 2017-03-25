#ifndef RGBDREADERINTERFACE_H
#define RGBDREADERINTERFACE_H

#include <QImage>
#include "Share/project_common.h"
#include "Share/pose6dof.h"
#include "Share/exceptions.h"

class RgbdReaderInterface
{
public:
    RgbdReaderInterface() {}
    virtual ~RgbdReaderInterface() {}

    virtual void ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose)
    {
        ReadRgbdFrame(index, color, depth);
        ReadFramePose(index, pose);
    }
    virtual void ReadRgbdFrame(const int index, QImage& color, QImage& depth) = 0;
    virtual void ReadFramePose(const int index, Pose6dof& pose) {}
    virtual void ChangeInstance() {}
    virtual int GetLength() { return 0; }

    static QString dataRootPath;
    static QString curDatasetPath;
    static QString curOutputPath;
};

#endif // RGBDREADERINTERFACE_H
