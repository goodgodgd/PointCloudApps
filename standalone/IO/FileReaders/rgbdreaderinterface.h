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

    virtual void ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose) {}
    virtual void ChangeInstance() {}

    static QString dsroot;
    static QString datasetPath;
};

#endif // RGBDREADERINTERFACE_H
