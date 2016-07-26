#ifndef TUMREADER_H
#define TUMREADER_H

#include "rgbdposereader.h"

struct RgbdPoseTuple
{
    double time;
    QString colorFile;
    QString depthFile;
    Pose6dof pose;
};

class TumReader : public RgbdPoseReader
{
public:
    TumReader(const int DSID_);

protected:
    virtual void LoadInitInfo(const int DSID);
    virtual QString ColorName(const int index);
    virtual QString DepthName(const int index);
    virtual Pathmap DatasetPath(const int DSID);
    virtual Pose6dof ReadPose(const int index);

private:
    std::vector<RgbdPoseTuple> LoadRgbdPoseTuples(Pathmap dataPaths);
    std::vector<RgbdPoseTuple> LoadOnlyDepth(const QString depthLogFileName);
    void FillInColorFile(const QString colorLogFileName, std::vector<RgbdPoseTuple>& tuples);
    void FillInPose(const QString trajFileName, std::vector<RgbdPoseTuple>& tuples);
    Pose6dof ConvertToPose(const QStringList& timePose);

    std::vector<Pose6dof> LoadTrajectory(const QString trajfile);
    Eigen::RowVector4f ReadRow(QString line);

    std::vector<RgbdPoseTuple> tuples;
};

#endif // TUMREADER_H
