#ifndef RGBDASYNCREADER_H
#define RGBDASYNCREADER_H

#include "rgbdposereader.h"

struct RgbDepthPair
{
    double time;
    QString colorFile;
    QString depthFile;
    Pose6dof pose;
};

class RgbdAsyncReader : public RgbdPoseReader
{
public:
    RgbdAsyncReader(const QString localPath);

private:
    void LoadInitInfo(const QString datapath);
    void WriteDepthListInText(const QString datapath);
    void ReadFramePose(const int index, Pose6dof& pose);
    QString ColorName(const int index);
    QString DepthName(const int index);

    std::vector<RgbDepthPair> LoadOnlyDepth(const QString depthLogFileName);
    void FillInColorFile(const QString colorLogFileName, std::vector<RgbDepthPair>& tuples);
    std::vector<Pose6dof> LoadTrajectory(const QString trajFileName, std::vector<RgbDepthPair>& tuples);
    Pose6dof ConvertToPose(const QStringList& timePose);
    void DrawTrajectory(const std::vector<Pose6dof>& trajectory, const int fromIndex);

    std::vector<RgbDepthPair> tuples;
    std::vector<Pose6dof> trajectory;
};

#endif // RGBDASYNCREADER_H
