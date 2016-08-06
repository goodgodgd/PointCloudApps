#ifndef RGBDREADERINTERFACE_H
#define RGBDREADERINTERFACE_H

#include <QImage>
#include "Share/project_common.h"
#include "Share/pose6dof.h"
#include "Share/exceptions.h"

class RgbdReaderInterface
{
public:
    RgbdReaderInterface() { qDebug() << "RgbdReaderInterface constructor"; }
    virtual ~RgbdReaderInterface() {}

    virtual void ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose) {}
    virtual void ChangeInstance() {}
};

#endif // RGBDREADERINTERFACE_H
