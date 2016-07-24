#ifndef RGBDPOSEREADER_H
#define RGBDPOSEREADER_H

#include <stdio.h>
#include <map>
#include <QStringList>
#include <QImage>
#include <QFile>
#include <QTextStream>
#include <opencv2/opencv.hpp>
#include "Share/project_common.h"
#include "Share/pose6dof.h"
#include "Share/exceptions.h"

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
        DSetEnd
    };
}

typedef ushort  DepthType;
typedef std::map<QString, QString> Pathmap;

class RgbdPoseReader
{
public:
    RgbdPoseReader(const int DSID_);
    virtual ~RgbdPoseReader() {}
    void ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose);

protected:
    virtual void LoadInitInfo(const int DSID) { qDebug() << "LoadInitInfo 1"; }
    virtual QString ColorName(const int index) = 0;
    virtual QString DepthName(const int index) = 0;
    virtual Pathmap DatasetPath(const int DSID) = 0;
    virtual Pose6dof ReadPose(const int index) = 0;

    QImage ReadColor(const QString name);
    QImage ReadDepth(const QString name);
    Pathmap dspaths;
    const int DSID;

    static constexpr char* dsroot = "/home/hyukdoo/Work/PointCloudApps/dataset";
    static constexpr char* keyColorPath = "color";
    static constexpr char* keyDepthPath = "depth";
    static constexpr char* keyTrajFile = "trajfile";

};

#endif // RGBDPOSEREADER_H
