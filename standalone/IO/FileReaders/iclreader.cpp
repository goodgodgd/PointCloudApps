#include "iclreader.h"

ICLReader::ICLReader(const int DSID_)
    : RgbdPoseReader(DSID_)
{
    LoadInitInfo(DSID);
}

void ICLReader::LoadInitInfo(const int DSID)
{
    dataPaths = DatasetPath(DSID);
    trajectory = LoadTrajectory(dataPaths[keyTrajFile]);
    qDebug() << "trajectory size" << trajectory.size();
}

Pathmap ICLReader::DatasetPath(const int DSID)
{
    QString dsetroot = QString(dsroot);
    Pathmap dataPaths;

    if(DSID==DSetID::ICL_NUIM_room1)
    {
        dsetPath = dsetroot + QString("/icl-nuim-livingroom1");
        dataPaths[keyColorPath] = dsetPath + QString("/livingroom1-color");
        dataPaths[keyDepthPath] = dsetPath + QString("/livingroom1-depth-clean");
        dataPaths[keyTrajFile] = dsetPath + QString("/livingroom1-traj.txt");
    }
    else if(DSID==DSetID::ICL_NUIM_room1_noisy)
    {
        dsetPath = dsetroot + QString("/icl-nuim-livingroom1");
        dataPaths[keyColorPath] = dsetPath + QString("/livingroom1-color");
        dataPaths[keyDepthPath] = dsetPath + QString("/livingroom1-depth-simulated");
        dataPaths[keyTrajFile] = dsetPath + QString("/livingroom1-traj.txt");
    }
    else if(DSID==DSetID::ICL_NUIM_office1)
    {
        dsetPath = dsetroot + QString("/icl-nuim-office1");
        dataPaths[keyColorPath] = dsetPath + QString("/office1-color");
        dataPaths[keyDepthPath] = dsetPath + QString("/office1-depth-clean");
        dataPaths[keyTrajFile] = dsetPath + QString("/office1-traj.txt");
    }
    else if(DSID==DSetID::ICL_NUIM_office1_noisy)
    {
        dsetPath = dsetroot + QString("/icl-nuim-office1");
        dataPaths[keyColorPath] = dsetPath + QString("/office1-color");
        dataPaths[keyDepthPath] = dsetPath + QString("/office1-depth-simulated");
        dataPaths[keyTrajFile] = dsetPath + QString("/office1-traj.txt");
    }
    else
        throw TryFrameException("wrong DSID for ICLReader");

    qDebug() << "trajectory file" << dataPaths[keyTrajFile];

    return dataPaths;
}

std::vector<Pose6dof> ICLReader::LoadTrajectory(const QString trajfile)
{
    QFile file(trajfile);
    if(file.open(QIODevice::ReadOnly)==false)
        throw TryFrameException("cannot open trajectory file");
    QTextStream reader(&file);

    std::vector<Pose6dof> trajectory;
    Eigen::Affine3f affine;
    Pose6dof pose;
    int count=0;

    while(!reader.atEnd())
    {
        QString strIndex = reader.readLine();
        QStringList indexList = strIndex.split(" ");
        if(indexList.at(0).toInt() != count++ || indexList.size() != 3)
            throw TryFrameException(QString("trajectory index error") + indexList.at(0));

        Eigen::Matrix4f mat;
        for(int i=0; i<4; i++)
        {
            if(reader.atEnd())
                throw TryFrameException(QString("trajectory file has invalid size"));
            mat.row(i) = ReadRow(reader.readLine());
        }
        affine.matrix() = mat;
        pose.SetPose6Dof(affine);
        trajectory.push_back(pose);
    }

    if(trajectory.size() < 2)
        throw TryFrameException(QString("trajectory is too short"));

    Eigen::Matrix4f rowExchanger = Eigen::Matrix4f::Zero();
    rowExchanger(0,1) = -1; // -Y -> X
    rowExchanger(1,2) = -1; // -Z -> Y
    rowExchanger(2,0) = -1; // -X -> Z
    rowExchanger(3,3) =  1;
    affine.matrix() = rowExchanger;
    Pose6dof poseConversion;
    poseConversion.SetPose6Dof(affine);

    for(Pose6dof& pose : trajectory)
        pose = pose * poseConversion;

    Pose6dof initialPose = trajectory[1];
    for(Pose6dof& pose : trajectory)
        pose = initialPose / pose;

    return trajectory;
}

Eigen::RowVector4f ICLReader::ReadRow(QString line)
{
    Eigen::RowVector4f row;
    QStringList list = line.split(" ");
    for(int i=0; i<4; i++)
        row(i) = (float)list.at(i).toDouble();
    return row;
}

QString ICLReader::ColorName(const int index)
{
    return dataPaths[keyColorPath] + QString("/%1.jpg").arg(index, 5, 10, QChar('0'));
}

QString ICLReader::DepthName(const int index)
{
    return dataPaths[keyDepthPath] + QString("/%1.png").arg(index, 5, 10, QChar('0'));
}
