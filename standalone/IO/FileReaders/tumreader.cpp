#include "tumreader.h"

TumReader::TumReader(const int DSID_)
    : RgbdPoseReader(DSID_)
{
    LoadInitInfo(DSID);
}

void TumReader::LoadInitInfo(const int DSID)
{
    dataPaths = DatasetPath(DSID);
    tuples = ListRgbdFiles(dataPaths);
    trajectory = LoadTrajectory(dataPaths[keyTrajFile], tuples);
    SaveImageTraj(tuples, trajectory);
    qDebug() << "trajectory size" << tuples.size();
}

Pathmap TumReader::DatasetPath(const int DSID)
{
    QString dsetroot = QString(dsroot);
    Pathmap dataPaths;

    if(DSID==DSetID::TUM_freiburg1_desk)
        datasetPath = dsetroot + QString("/tum_freiburg1_desk");
    else if(DSID==DSetID::TUM_freiburg1_room)
        datasetPath = dsetroot + QString("/tum_freiburg1_room");
    else if(DSID==DSetID::TUM_freiburg2_desk)
        datasetPath = dsetroot + QString("/tum_freiburg2_desk");
    else if(DSID==DSetID::TUM_freiburg3_long)
        datasetPath = dsetroot + QString("/tum_freiburg3_long");
    else if(DSID==DSetID::Corbs_cabinet)
        datasetPath = dsetroot + QString("/corbs-cabinet2");
    else if(DSID==DSetID::Corbs_desk)
        datasetPath = dsetroot + QString("/corbs-desk2");
    else if(DSID==DSetID::Corbs_human)
        datasetPath = dsetroot + QString("/corbs-human2");
    else
        throw TryFrameException(QString("wrong DSID for TumReader %1").arg(DSID));

    dataPaths[keyColorPath] = datasetPath;
    dataPaths[keyDepthPath] = datasetPath;
    dataPaths[keyTrajFile] = datasetPath + QString("/groundtruth.txt");

    qDebug() << "trajectory file" << dataPaths[keyTrajFile];

    return dataPaths;
}

std::vector<RgbDepthPair> TumReader::ListRgbdFiles(Pathmap dataPaths)
{
    std::vector<RgbDepthPair> tuples = LoadOnlyDepth(dataPaths[keyDepthPath] + QString("/depth.txt"));
    FillInColorFile(dataPaths[keyColorPath] + QString("/rgb.txt"), tuples);
    return tuples;
}

std::vector<RgbDepthPair> TumReader::LoadOnlyDepth(const QString depthLogFileName)
{
    QFile depthLog(depthLogFileName);
    if(depthLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException("cannot open depth file");
    QTextStream reader(&depthLog);

    std::vector<RgbDepthPair> tuples;
    RgbDepthPair sgtuple;
    while(!reader.atEnd())
    {
        QString line = reader.readLine();
        if(line.startsWith("#"))
            continue;

        QStringList timeFile = line.split(" ");
        if(timeFile.size() != 2)
            throw TryFrameException("invalid depth line");

        sgtuple.time = timeFile.at(0).toDouble();
        sgtuple.depthFile = timeFile.at(1);
        sgtuple.colorFile = "";
        sgtuple.pose = Pose6dof();

        tuples.push_back(sgtuple);
    }

    return tuples;
}

void TumReader::FillInColorFile(const QString colorLogFileName, std::vector<RgbDepthPair>& tuples)
{
    QFile colorLog(colorLogFileName);
    if(colorLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException("cannot open color file");
    QTextStream reader(&colorLog);

    double curTime, befTime=0.0;
    QString curFile, befFile;
    int tpIdx=0;

    while(!reader.atEnd())
    {
        QString line = reader.readLine();
        if(line.startsWith("#"))
            continue;

        QStringList timeFile = line.split(" ");
        if(timeFile.size() != 2)
            throw TryFrameException("invalid color line");

        curTime = timeFile.at(0).toDouble();
        curFile = timeFile.at(1);

        while(tpIdx < tuples.size() && tuples[tpIdx].time >= befTime && tuples[tpIdx].time <= curTime)
        {
            if(tuples[tpIdx].time - befTime > curTime - tuples[tpIdx].time)
                tuples[tpIdx].colorFile = curFile;
            else
                tuples[tpIdx].colorFile = befFile;
            tpIdx++;
        }

        befTime = curTime;
        befFile = curFile;
    }
}

std::vector<Pose6dof> TumReader::LoadTrajectory(const QString trajFileName, std::vector<RgbDepthPair>& tuples)
{
    QFile trajLog(trajFileName);
    if(trajLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException("cannot open trajectory file");
    QTextStream reader(&trajLog);
    std::vector<Pose6dof> trajectory(tuples.size());

    double curTime, befTime=0.0;
    Pose6dof curPose, befPose;
    int tpIdx=0;

    while(!reader.atEnd())
    {
        QString line = reader.readLine();
        if(line.startsWith("#"))
            continue;

        QStringList timePose = line.split(" ");
        if(timePose.size() != 8)
            throw TryFrameException("invalid pose line");

        curTime = timePose.at(0).toDouble();
        curPose = ConvertToPose(timePose);

        while(tpIdx < tuples.size() && tuples[tpIdx].time >= befTime && tuples[tpIdx].time <= curTime)
        {
            if(tuples[tpIdx].time - befTime > curTime - tuples[tpIdx].time)
                trajectory[tpIdx] = curPose;
            else
                trajectory[tpIdx] = befPose;
            tpIdx++;
        }

        befTime = curTime;
        befPose = curPose;
    }

    qDebug() << "1  " << trajectory[1];
    qDebug() << "60 " << trajectory[60];
    qDebug() << "rel" << trajectory[1] / trajectory[60];

    Eigen::Matrix4f rowExchanger = Eigen::Matrix4f::Zero();
    rowExchanger(0,1) = -1; // -Y -> X
    rowExchanger(1,2) = -1; // -Z -> Y
    rowExchanger(2,0) = 1; //  X -> Z
    rowExchanger(3,3) =  1;
    Eigen::Affine3f affine;
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

void TumReader::SaveImageTraj(const std::vector<RgbDepthPair>& tuples, const std::vector<Pose6dof>& trajectory)
{
    if(tuples.size() != trajectory.size())
        throw TryFrameException("different sizes of tuple and trajectory");

    SaveDepthList(dataPaths[keyDepthPath] + QString("/depthList.txt"), tuples);
    SaveTrajectory(dataPaths[keyDepthPath] + QString("/trajectory.txt"), trajectory);
}

void TumReader::SaveDepthList(const QString fileName, const std::vector<RgbDepthPair>& tuples)
{
    QFile depthFile(fileName);
    if(depthFile.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw TryFrameException("cannot open depthList file");
    QTextStream writer(&depthFile);
    for(const RgbDepthPair& rgbd : tuples)
        writer << rgbd.depthFile << "\n";
    depthFile.close();
}

void TumReader::SaveTrajectory(const QString fileName, const std::vector<Pose6dof>& trajectory)
{
    QFile trajFile(fileName);
    if(trajFile.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw TryFrameException("cannot open trajectory file");
    QTextStream writer(&trajFile);
    for(const Pose6dof& pose : trajectory)
    {
        for(int i=0; i<7; i++)
            writer << pose.data[i] << " ";
        writer << "\n";
    }
    trajFile.close();
}

Pose6dof TumReader::ConvertToPose(const QStringList& timePose)
{
    Eigen::VectorXf posevec = Eigen::VectorXf::Zero(7);
    for(int i=0; i<3; i++)
        posevec(i) = (float)timePose.at(i+1).toDouble();
    posevec(3) = (float)timePose.at(6+1).toDouble();
    for(int i=4; i<7; i++)
        posevec(i) = (float)timePose.at(i).toDouble();
    Pose6dof pose;
    pose.SetPose6Dof(posevec);
    return pose;
}

QString TumReader::ColorName(const int index)
{
    return dataPaths[keyColorPath] + QString("/") + tuples[index].colorFile;
}

QString TumReader::DepthName(const int index)
{
    return dataPaths[keyDepthPath] + QString("/") + tuples[index].depthFile;
}
