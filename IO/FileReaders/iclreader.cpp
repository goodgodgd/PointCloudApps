#include "iclreader.h"

ICLReader::ICLReader(const int DSID_)
    : RgbdPoseReader(DSID_)
{
    LoadInitInfo(DSID);
}

void ICLReader::LoadInitInfo(const int DSID)
{
    dspaths = DatasetPath(DSID);
    trajectory = LoadTrajectory(dspaths[keyTrajFile]);
    qDebug() << "trajectory size" << trajectory.size();
}

Pathmap ICLReader::DatasetPath(const int DSID)
{
    QString dsetroot = QString(dsroot);
    Pathmap dsetpaths;

    if(DSID==DSetID::ICL_NUIM_room1)
    {
        dsetpaths[keyColorPath] = dsetroot + QString("/icl-nuim-livingroom1/livingroom1-color");
        dsetpaths[keyDepthPath] = dsetroot + QString("/icl-nuim-livingroom1/livingroom1-depth-clean");
        dsetpaths[keyTrajFile] = dsetroot + QString("/icl-nuim-livingroom1/livingroom1-traj.txt");
    }
    else if(DSID==DSetID::ICL_NUIM_room1_noisy)
    {
        dsetpaths[keyColorPath] = dsetroot + QString("/icl-nuim-livingroom1/livingroom1-color");
        dsetpaths[keyDepthPath] = dsetroot + QString("/icl-nuim-livingroom1/livingroom1-depth-simulated");
        dsetpaths[keyTrajFile] = dsetroot + QString("/icl-nuim-livingroom1/livingroom1-traj.txt");
    }
    else if(DSID==DSetID::ICL_NUIM_office1)
    {
        dsetpaths[keyColorPath] = dsetroot + QString("/icl-nuim-office1/office1-color");
        dsetpaths[keyDepthPath] = dsetroot + QString("/icl-nuim-office1/office1-depth-clean");
        dsetpaths[keyTrajFile] = dsetroot + QString("/icl-nuim-office1/office1-traj.txt");
    }
    else if(DSID==DSetID::ICL_NUIM_office1_noisy)
    {
        dsetpaths[keyColorPath] = dsetroot + QString("/icl-nuim-office1/office1-color");
        dsetpaths[keyDepthPath] = dsetroot + QString("/icl-nuim-office1/office1-depth-simulated");
        dsetpaths[keyTrajFile] = dsetroot + QString("/icl-nuim-office1/office1-traj.txt");
    }
    else
        throw TryFrameException("wrong DSID for ICLReader");

    qDebug() << "trajectory file" << dsetpaths[keyTrajFile];

    return dsetpaths;
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
//        if(count<10 || count%100==0)
//            qDebug() << count << pose;
    }
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
    return dspaths[keyColorPath] + QString("/%1.jpg").arg(index, 5, 10, QChar('0'));
}

QString ICLReader::DepthName(const int index)
{
    return dspaths[keyDepthPath] + QString("/%1.png").arg(index, 5, 10, QChar('0'));
}

Pose6dof ICLReader::ReadPose(const int index)
{
    static Pose6dof tfmPose;
    static bool tfmInit = false;
    if(tfmInit==false)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
        transform(0,1) = -1; // -Y -> X
        transform(1,2) = -1; // -Z -> Y
        transform(2,0) =  1; //  X -> Z
        transform(3,3) = 1;
        Eigen::Affine3f affine;
        affine.matrix() = transform;
        tfmPose.SetPose6Dof(affine);
        tfmInit = true;
        std::cout << "transform" << std::endl << transform << std::endl;
        qDebug() << "tfmPose" << tfmPose;
    }

    if(trajectory.size() <= index)
        throw TryFrameException(QString("pose index is out of size %1<=%2").arg((int)trajectory.size()).arg(index));
    return tfmPose*trajectory[index];
}
