#ifndef RGBDPOSEREADER_H
#define RGBDPOSEREADER_H

#include <stdio.h>
#include <map>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <opencv2/opencv.hpp>
#include "rgbdreaderinterface.h"

namespace DSetID {
    enum Enum
    {
        ICL_NUIM_room1 = 0,
        ICL_NUIM_room1_noisy,
        ICL_NUIM_office1,
        ICL_NUIM_office1_noisy,
        TUM_freiburg1_desk,
        TUM_freiburg1_room,
        TUM_freiburg2_desk,
        TUM_freiburg3_long,
        Rgbd_Objects,
        DSetEnd,
    };
}

typedef ushort  DepthType;
typedef std::map<int, QString> Pathmap;

class RgbdPoseReader : public RgbdReaderInterface
{
public:
    RgbdPoseReader(const int DSID_);
    virtual ~RgbdPoseReader() {}
    virtual void ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose);

    enum Enum
    {
        keyColorPath,
        keyDepthPath,
        keyTrajFile
    };

    static constexpr char* dsroot = "/media/hyukdoo/Edrive/PaperData/rgbd-scene-dataset";
    static QString dsetPath;
    static int DSID;

protected:
    virtual void LoadInitInfo(const int DSID) { qDebug() << "LoadInitInfo 1"; }
    virtual QString ColorName(const int index) = 0;
    virtual QString DepthName(const int index) = 0;
    virtual Pathmap DatasetPath(const int DSID) = 0;
    virtual Pose6dof ReadPose(const int index) = 0;

    QImage ReadColor(const QString name);
    QImage ReadDepth(const QString name);
    Pathmap dataPaths;
};

#endif // RGBDPOSEREADER_H
