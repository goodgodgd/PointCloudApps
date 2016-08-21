#ifndef ICLREADER_H
#define ICLREADER_H

#include <iostream>
#include "rgbdposereader.h"
#include "IO/glvertexmanager.h"

class ICLReader : public RgbdPoseReader
{
public:
    ICLReader(const int DSID_);

protected:
    virtual void LoadInitInfo(const int DSID);
    virtual QString ColorName(const int index);
    virtual QString DepthName(const int index);
    virtual Pathmap DatasetPath(const int DSID);

private:
    std::vector<Pose6dof> LoadTrajectory(const QString trajfile);
    Eigen::RowVector4f ReadRow(QString line);
};

#endif // ICLREADER_H
