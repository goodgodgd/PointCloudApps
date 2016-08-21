#ifndef ICLREADER_H
#define ICLREADER_H

#include <iostream>
#include "rgbdposereader.h"
#include "IO/glvertexmanager.h"

//========== ICL coordinate system ==========
// X: right = -Y in my system
// Y: up    = Z in my system
// Z: depth = X in my system
//===========================================

class ICLReader : public RgbdPoseReader
{
public:
    ICLReader(const int DSID_);

protected:
    virtual void LoadInitInfo(const int DSID);
    virtual QString ColorName(const int index);
    virtual QString DepthName(const int index);
    virtual Pathmap DatasetPath(const int DSID);
    virtual Pose6dof ReadPose(const int index);

private:
    std::vector<Pose6dof> LoadTrajectory(const QString trajfile);
    Eigen::RowVector4f ReadRow(QString line);

    std::vector<Pose6dof> trajectory;
};

#endif // ICLREADER_H
