#include "tumreader.h"

TumReader::TumReader(const int DSID_)
    : RgbdPoseReader(DSID_)
{
    LoadInitInfo(DSID);
}

void TumReader::LoadInitInfo(const int DSID)
{
    dspaths = DatasetPath(DSID);
    tuples = LoadRgbdPoseTuples(dspaths);
    qDebug() << "trajectory size" << tuples.size();
}

Pathmap TumReader::DatasetPath(const int DSID)
{
    QString dsetroot = QString(dsroot);
    Pathmap dsetpaths;

    if(DSID==DSetID::TUM_freiburg1_desk)
    {
        dsetpaths[keyColorPath] = dsetroot + QString("/tum_freiburg1_desk");
        dsetpaths[keyDepthPath] = dsetroot + QString("/tum_freiburg1_desk");
        dsetpaths[keyTrajFile] = dsetroot + QString("/tum_freiburg1_desk/groundtruth.txt");
    }
    else
        throw TryFrameException("wrong DSID for TumReader");

    qDebug() << "trajectory file" << dsetpaths[keyTrajFile];

    return dsetpaths;
}

std::vector<RgbdPoseTuple> TumReader::LoadRgbdPoseTuples(Pathmap dspaths)
{
    std::vector<RgbdPoseTuple> tuples = LoadOnlyDepth(dspaths[keyDepthPath] + QString("/depth.txt"));
    FillInColorFile(dspaths[keyColorPath] + QString("/rgb.txt"), tuples);
    FillInPose(dspaths[keyTrajFile], tuples);

//    for(int i=0; i<tuples.size(); i+=100)
//    {
//        qDebug() << "tuple" << tuples[i].time << tuples[i].colorFile << tuples[i].depthFile << tuples[i].pose;
//        qDebug() << "tuple" << tuples[i+1].time << tuples[i+1].colorFile << tuples[i+1].depthFile << tuples[i+1].pose;
//    }
    return tuples;
}

std::vector<RgbdPoseTuple> TumReader::LoadOnlyDepth(const QString depthLogFileName)
{
    QFile depthLog(depthLogFileName);
    if(depthLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException("cannot open depth file");
    QTextStream reader(&depthLog);

    std::vector<RgbdPoseTuple> tuples;
    RgbdPoseTuple sgtuple;
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

void TumReader::FillInColorFile(const QString colorLogFileName, std::vector<RgbdPoseTuple>& tuples)
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

void TumReader::FillInPose(const QString trajFileName, std::vector<RgbdPoseTuple>& tuples)
{
    QFile trajLog(trajFileName);
    if(trajLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException("cannot open trajectory file");
    QTextStream reader(&trajLog);

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
                tuples[tpIdx].pose = curPose;
            else
                tuples[tpIdx].pose = befPose;
            tpIdx++;
        }

        befTime = curTime;
        befPose = curPose;
    }
}

Pose6dof TumReader::ConvertToPose(const QStringList& timePose)
{
    Eigen::VectorXf posevec = Eigen::VectorXf::Zero(7);
    for(int i=0; i<7; i++)
        posevec(i) = (float)timePose.at(i+1).toDouble();
    Pose6dof pose;
    pose.SetPose6Dof(posevec);
    return pose;
}

QString TumReader::ColorName(const int index)
{
    return dspaths[keyColorPath] + QString("/") + tuples[index].colorFile;
}

QString TumReader::DepthName(const int index)
{
    return dspaths[keyDepthPath] + QString("/") + tuples[index].depthFile;
}

Pose6dof TumReader::ReadPose(const int index)
{
    if(tuples.size() <= index)
        throw TryFrameException(QString("pose index is out of size %1<=%2").arg((int)tuples.size()).arg(index));
    return tuples[index].pose;
}
