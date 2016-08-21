#include "trackrecorder.h"

TrackRecorder::TrackRecorder()
{
}

void TrackRecorder::Record(const std::vector<TrackPoint>* trackPoints_
                                , const DescType* cwg_
                                , pcl::PointCloud<SpinImageType>::Ptr spin_
                                , pcl::PointCloud<FPFHType>::Ptr fpfh_
                                , pcl::PointCloud<SHOTType>::Ptr shot_
                                )
{
    trackPoints = trackPoints_;
    cwg = cwg_;
    spin = spin_;
    fpfh = fpfh_;
    shot = shot_;
//    narf = narf_;

    try {
        CheckLengths();
        QString fileName = CreatePathAndFile("Descriptor1", "DDS");
        RecordDescriptors(fileName);
        qDebug() << "record completed";
    }
    catch (RecordException exception) {
        qDebug() << "RecordException:" << exception.msg;
    }
}

void TrackRecorder::CheckLengths()
{
    if(spin->points.size() != fpfh->points.size())
        throw RecordException("different descriptor size");
    if(spin->points.size() != shot->points.size())
        throw RecordException("different descriptor size");
//    if(trackPoints->size() != narf->points.size())
//        throw RecordException("narf size is not valid");
}

QString TrackRecorder::CreatePathAndFile(const QString dirName, const QString filePrefix)
{
    if(g_frameIdx==1)
    {
        dstPath = RgbdPoseReader::dsetPath + QString("/") + dirName;

        if(RgbdPoseReader::DSID == DSetID::ICL_NUIM_room1_noisy)
            dstPath.replace(QString("icl-nuim-livingroom1"), QString("icl-nuim-livingroom1-noisy"));
        if(RgbdPoseReader::DSID == DSetID::ICL_NUIM_office1_noisy)
            dstPath.replace(QString("icl-nuim-office1"), QString("icl-nuim-office1-noisy"));

        QDir dir;
        if(!dir.exists(dstPath))
            if(!dir.mkdir(dstPath))
                throw RecordException("failed to create directory");
    }

    QString fileName = dstPath + QString("/") + filePrefix + QString("_%1.txt").arg(g_frameIdx, 5, 10, QChar('0'));
    return fileName;
}

void TrackRecorder::RecordDescriptors(QString fileName)
{
    QFile file(fileName);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw RecordException(QString("cannot create descriptor file ")+fileName);
    QTextStream writer(&file);

    int count=0;
    for(size_t i=0; i<trackPoints->size(); i++)
    {
        if(trackPoints->at(i).frameIndex != g_frameIdx)
            continue;
        cl_uint2 pixel = trackPoints->at(i).pixel;
        WriteTrackInfo(writer, trackPoints->at(i));
        WriteDescriptor(writer, cwg[PIXIDX(pixel)].s, DescSize);
        WriteDescriptor(writer, spin->at(count).histogram, SPIN_SIZE);
        WriteDescriptor(writer, fpfh->at(count).histogram, FPFHType::descriptorSize());
        WriteDescriptor(writer, shot->at(count).descriptor, SHOTType::descriptorSize());
//        WriteDescriptor(writer, narf->at(i).descriptor, 36);
        writer << '\n';
        ++count;
    }
    file.close();
}

void TrackRecorder::WriteTrackInfo(QTextStream& writer, const TrackPoint trackPoint)
{
    writer << trackPoint.ID << " " << trackPoint.frameIndex << " " << trackPoint.tcount << " " << trackPoint.pixel.x << " " << trackPoint.pixel.y << " "
               << qSetRealNumberPrecision(3) << trackPoint.lpoint.x<< " " << trackPoint.lpoint.y << " " << trackPoint.lpoint.z;
}

void TrackRecorder::WriteDescriptor(QTextStream& writer, const float* descriptor, const int size)
{
    for(int i=0; i<size; i++)
        writer << " " << descriptor[i];
}
