#include "trackrecorder.h"

TrackRecorder::TrackRecorder()
{
}

void TrackRecorder::Record(SharedData* shdDat
                                , const std::vector<TrackPoint>* trackPoints_
                                , pcl::PointCloud<SpinImageType>::Ptr spin_
                                , pcl::PointCloud<FPFHType>::Ptr fpfh_
                                , pcl::PointCloud<SHOTType>::Ptr shot_
                                , pcl::PointCloud<TrisiType>::Ptr trisi_
                                )
{
    pointCloud = shdDat->ConstPointCloud();
    normalCloud = shdDat->ConstNormalCloud();
    praxesCloud = shdDat->ConstPrinAxes();
    trackPoints = trackPoints_;
    pcwg = shdDat->ConstDescriptors();
    spin = spin_;
    fpfh = fpfh_;
    shot = shot_;
    trisi = trisi_;
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
    if(fpfh->points.size() != shot->points.size())
        throw RecordException("different descriptor size");
    if(fpfh->points.size() != trisi->points.size())
        throw RecordException("different descriptor size");
//    if(trackPoints->size() != narf->points.size())
//        throw RecordException("narf size is not valid");
}

QString TrackRecorder::CreatePathAndFile(const QString dirName, const QString filePrefix)
{
    if(g_frameIdx==1)
    {
        int radius = (int)(DESC_RADIUS*100.f);
        dstPath = RgbdReaderInterface::datasetPath + QString("/DescriptorR%1").arg(radius);

//        if(RgbdPoseReader::DSID == DSetID::ICL_NUIM_room1_noisy)
//            dstPath.replace(QString("icl-nuim-livingroom1"), QString("icl-nuim-livingroom1-noisy"));
//        if(RgbdPoseReader::DSID == DSetID::ICL_NUIM_office1_noisy)
//            dstPath.replace(QString("icl-nuim-office1"), QString("icl-nuim-office1-noisy"));

        QDir dir;
        if(!dir.exists(dstPath))
            if(!dir.mkpath(dstPath))
                throw RecordException("failed to create directory");
    }

    QString fileName = dstPath + QString("/DDS_%1.txt").arg(g_frameIdx, 5, 10, QChar('0'));
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
        WriteDescriptor(writer, pcwg[PIXIDX(pixel)].s, DescSize);
//        WriteDescriptor(writer, spin->at(count).histogram, SpinImageType::descriptorSize());
        WriteDescriptor(writer, fpfh->at(count).histogram, FPFHType::descriptorSize());
        WriteDescriptor(writer, shot->at(count).descriptor, SHOTType::descriptorSize());
        WriteDescriptor(writer, trisi->at(count).histogram, TrisiType::descriptorSize());
//        WriteDescriptor(writer, narf->at(i).descriptor, 36);
        writer << '\n';
        ++count;
    }
    file.close();
}

void TrackRecorder::WriteTrackInfo(QTextStream& writer, const TrackPoint trackPoint)
{
    const int pxidx = PIXIDX(trackPoint.pixel);
    const int pxidx2 = IMGIDX(trackPoint.pixel.y, trackPoint.pixel.x);
    assert(pxidx==pxidx2);
    assert(!clIsNull(pointCloud[pxidx]));
    assert(!clIsNull(normalCloud[pxidx]));
    assert(!clIsNull(praxesCloud[pxidx]));
//    qDebug() << "trackpoint" << trackPoint.ID << trackPoint.pixel << pointCloud[pxidx] << "pxidx" << pxidx << pxidx2;
    writer << trackPoint.ID << " " << trackPoint.pixel.x << " " << trackPoint.pixel.y << " "
                << qSetRealNumberPrecision(4) << pointCloud[pxidx].x << " " << pointCloud[pxidx].y << " " << pointCloud[pxidx].z << " "
                    << normalCloud[pxidx].x << " " << normalCloud[pxidx].y << " " << normalCloud[pxidx].z << " "
                        << praxesCloud[pxidx].s[0] << " " << praxesCloud[pxidx].s[1] << " " << praxesCloud[pxidx].s[2];
}

void TrackRecorder::WriteDescriptor(QTextStream& writer, const float* descriptor, const int size)
{
    for(int i=0; i<size; i++)
        writer << " " << descriptor[i];
}
