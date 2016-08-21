#ifndef TUMREADER_H
#define TUMREADER_H

#include "rgbdposereader.h"

//========== ICL coordinate system ==========
// It uses left-handed coordinate system
// X: right = -Y in my system
// Y: up    = Z in my system
// Z: depth = X in my system
//===========================================

struct RgbDepthPair
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

private:
    std::vector<RgbDepthPair> ListRgbdFiles(Pathmap dataPaths);
    std::vector<RgbDepthPair> LoadOnlyDepth(const QString depthLogFileName);
    void FillInColorFile(const QString colorLogFileName, std::vector<RgbDepthPair>& tuples);
    std::vector<Pose6dof> LoadTrajectory(const QString trajFileName, std::vector<RgbDepthPair>& tuples);
    Pose6dof ConvertToPose(const QStringList& timePose);
    Eigen::RowVector4f ReadRow(QString line);

    std::vector<RgbDepthPair> tuples;
};

#endif // TUMREADER_H
