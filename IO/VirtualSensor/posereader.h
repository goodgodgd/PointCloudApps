#ifndef POSEREADER_H
#define POSEREADER_H

#include <QMatrix4x4>
#include <QVector3D>
#include "Share/project_common.h"
#include "readerutil.h"

class PoseReader
{
    enum Enum
    {
        NUM_ATTRIB = 2
    };

public:
    PoseReader() {}
    static QMatrix4x4 ReadPose(const QString filename)
    {
        QFile file(filename);
        if(file.open(QIODevice::ReadOnly)==false)
            throw QString("pose file not opened");
        QTextStream reader(&file);
        MapNameData attribMap = ReaderUtil::ReadAttributes(reader, NUM_ATTRIB);

        QStringList attribList;
        attribList << "position" << "rpy_deg";
        assert(attribList.size()==NUM_ATTRIB);
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        QVector3D position, eulerDeg;
        position << attribMap[attribList[0]].GetVector();
        eulerDeg << attribMap[attribList[1]].GetVector();

        QMatrix4x4 viewPose;
        viewPose.setToIdentity();
        viewPose.translate(position);
        // rotation must be commited in yaw - pitch - roll order
        viewPose.rotate(eulerDeg.z(), QVector3D(0,0,1));
        viewPose.rotate(eulerDeg.y(), QVector3D(0,1,0));
        viewPose.rotate(eulerDeg.x(), QVector3D(1,0,0));

        qDebug() << "read pose: position" << position << "angle" << eulerDeg << "x-axis" << viewPose.mapVector(QVector3D(1,0,0));
//        qDebug() << "pose matrix" << viewPose;
        return viewPose;
    }
};

#endif // POSEREADER_H
