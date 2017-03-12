#include "rgbdasyncreader.h"

RgbdAsyncReader::RgbdAsyncReader(const QString localPath)
    : RgbdPoseReader(localPath)
{
    LoadInitInfo(curDatasetPath);
    WriteDepthListInText(curDatasetPath);
    indexScale = smax(tuples.size()/500, 1);
    if(curDatasetPath.contains("CoRBS"))
        depthScale = 5;
}

void RgbdAsyncReader::LoadInitInfo(const QString datapath)
{
    tuples = LoadOnlyDepth(curDatasetPath + QString("/depth.txt"));
    FillInColorFile(curDatasetPath + QString("/rgb.txt"), tuples);
    trajectory = LoadTrajectory(curDatasetPath + QString("/groundtruth.txt"), tuples);
    qDebug() << "RgbdAsyncReader: trajectory size" << tuples.size();
}

void RgbdAsyncReader::WriteDepthListInText(const QString datapath)
{
    int radius = (int)(DESC_RADIUS*100.f);
    QString dstDir = QString("DescriptorR%1").arg(radius);
    QDir dir(datapath);
    if(!dir.exists(dstDir))
        if(!dir.mkdir(dstDir))
            throw TryFrameException("failed to create directory");

    QString filename = datapath + QString("/") + dstDir + QString("/depthList.txt");
    QFile depthListFile(filename);
    if(depthListFile.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw TryFrameException(QString("cannot create depth list file ")+filename);
    qDebug() << "depthListFile" << filename;

    QTextStream writer(&depthListFile);
    for(int i=0; i<tuples.size(); i++)
        writer << DepthName(i) << "\n";
}

void RgbdAsyncReader::ReadFramePose(const int index, Pose6dof& pose)
{
    if(trajectory.size() <= index)
        throw TryFrameException(QString("pose index is out of size %1<=%2").arg((int)trajectory.size()).arg(index));
    pose = trajectory[index];
    DrawTrajectory(trajectory, index);
}

QString RgbdAsyncReader::ColorName(const int index)
{
    int frameIndex = index * indexScale;
    if(frameIndex >= tuples.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(frameIndex).arg(tuples.size()));
    return curDatasetPath + QString("/") + tuples[frameIndex].colorFile;
}

QString RgbdAsyncReader::DepthName(const int index)
{
    int frameIndex = index * indexScale;
    if(frameIndex >= tuples.size())
        throw TryFrameException(QString("dataset finished, index out of range %1 > %2").arg(frameIndex).arg(tuples.size()));
    return curDatasetPath + QString("/") + tuples[frameIndex].depthFile;
}


std::vector<RgbDepthPair> RgbdAsyncReader::LoadOnlyDepth(const QString depthLogFileName)
{
    QFile depthLog(depthLogFileName);
    if(depthLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException(QString("cannot open depth file: ") + depthLogFileName);
    QTextStream reader(&depthLog);
    int count=0;

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
        sgtuple.depthFile.replace("\\", "/");
        sgtuple.colorFile = "";
        sgtuple.pose = Pose6dof();
        tuples.push_back(sgtuple);
    }

    return tuples;
}

void RgbdAsyncReader::FillInColorFile(const QString colorLogFileName, std::vector<RgbDepthPair>& tuples)
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
        curFile.replace("\\", "/");

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

std::vector<Pose6dof> RgbdAsyncReader::LoadTrajectory(const QString trajFileName, std::vector<RgbDepthPair>& tuples)
{
    QFile trajLog(trajFileName);
    if(trajLog.open(QIODevice::ReadOnly)==false)
        throw TryFrameException("cannot open trajectory file");
    QTextStream reader(&trajLog);
    std::vector<Pose6dof> trajectory(tuples.size());

    double curTime=10000000000.0, befTime=0.0;
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

Pose6dof RgbdAsyncReader::ConvertToPose(const QStringList& timePose)
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

void RgbdAsyncReader::DrawTrajectory(const std::vector<Pose6dof>& trajectory, const int fromIndex)
{
    cl_float4 vertex;
    cl_float4 color = (cl_float4){1,1,0,0};
    cl_float4 normal = (cl_float4){0,0,1,0};
    Pose6dof relPose;
    int drawUpto = (int)smin((size_t)(fromIndex+600), trajectory.size());
    for(int i=fromIndex; i<drawUpto; ++i)
    {
        relPose = trajectory[fromIndex] / trajectory[i];
        vertex = (cl_float4){relPose.x, relPose.y, relPose.z, 0};
        gvm::AddVertex(VertexType::line, vertex, color, normal, 2);

        relPose = trajectory[fromIndex] / trajectory[i+1];
        vertex = (cl_float4){relPose.x, relPose.y, relPose.z, 0};
        gvm::AddVertex(VertexType::line, vertex, color, normal, 2, true);
    }
}
