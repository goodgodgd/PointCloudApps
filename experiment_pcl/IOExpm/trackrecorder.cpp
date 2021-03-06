#include "trackrecorder.h"

TrackRecorder::TrackRecorder()
{
}

void TrackRecorder::Record(SharedData* shdDat,
                           const std::vector<TrackPoint>* trackPoints_,
                           pcl::PointCloud<SpinImageType>::Ptr spin_,
                           pcl::PointCloud<FPFHType>::Ptr fpfh_,
                           pcl::PointCloud<SHOTType>::Ptr shot_,
                           pcl::PointCloud<TrisiType>::Ptr trisi_,
                           bool bNewFile
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
#ifdef SCALE_VAR
        QString fileName = RgbdReaderInterface::curOutputPath + QString("/sampleScale%1.txt").arg(SCALE_VAR);
#else
        QString fileName = RgbdReaderInterface::curOutputPath + QString("/DDS_%1.txt").arg(g_frameIdx, 5, 10, QChar('0'));
#endif
        OpenFile(fileName, bNewFile);
        RecordDescriptors();
        qDebug() << "TrackRecorder:" << trackPoints->size() << "descriptors were written in" << fileName;
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
}

void TrackRecorder::OpenFile(QString fileName, bool bNewFile)
{
    static QString fileBefore;
    bool createFile = bNewFile;
    if(createFile==false && fileBefore.contains(fileName)==false)
        createFile = true;
    fileBefore = fileName;

    file.setFileName(fileName);
    if(createFile)
    {
        qDebug() << "create file" << fileName;
        if(file.open(QIODevice::WriteOnly | QIODevice::Text)==false)
            throw RecordException(QString("cannot create descriptor file ")+fileName);
    }
    else
    {
        qDebug() << "open file" << fileName;
        if(file.open(QIODevice::Append | QIODevice::Text)==false)
            throw RecordException(QString("cannot read descriptor file ")+fileName);
    }
}

void TrackRecorder::RecordDescriptors()
{
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
    if(clIsNull(pointCloud[pxidx]) || clIsNull(normalCloud[pxidx]) || clIsNull(praxesCloud[pxidx]))
    {
        qDebug() << "invalid track:" << pointCloud[pxidx] << normalCloud[pxidx] << praxesCloud[pxidx];
        throw TryFrameException("invalid track point");
    }
//    qDebug() << "trackpoint" << trackPoint.ID << trackPoint.pixel << pointCloud[pxidx] << "pxidx" << pxidx << pxidx2;
    writer << g_frameIdx << " " << trackPoint.pixel.x << " " << trackPoint.pixel.y << " "
                << qSetRealNumberPrecision(4) << pointCloud[pxidx].x << " " << pointCloud[pxidx].y << " " << pointCloud[pxidx].z << " "
                    << normalCloud[pxidx].x << " " << normalCloud[pxidx].y << " " << normalCloud[pxidx].z << " "
                        << praxesCloud[pxidx].s[0] << " " << praxesCloud[pxidx].s[1] << " " << praxesCloud[pxidx].s[2];
}

void TrackRecorder::WriteDescriptor(QTextStream& writer, const float* descriptor, const int size)
{
    for(int i=0; i<size; i++)
        writer << " " << descriptor[i];
}
