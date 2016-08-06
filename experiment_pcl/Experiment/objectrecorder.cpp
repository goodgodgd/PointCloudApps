#include "objectrecorder.h"

ObjectRecorder::ObjectRecorder()
{
}

void ObjectRecorder::Record(boost::shared_ptr<std::vector<int>> indicesptr_
                                , const DescType* cwg_
                                , pcl::PointCloud<SpinImageType>::Ptr spin_
                                , pcl::PointCloud<FPFHType>::Ptr fpfh_
                                , pcl::PointCloud<SHOTType>::Ptr shot_
                                )
{
    indicesptr = indicesptr_;
    cwg = cwg_;
    spin = spin_;
    fpfh = fpfh_;
    shot = shot_;
//    narf = narf_;

    try {
        CheckLengths();
        QString fileName = CreatePathAndFile("_descriptor1");
        RecordDescriptors(fileName);
        qDebug() << "record completed";
    }
    catch (RecordException exception) {
        qDebug() << "RecordException:" << exception.msg;
    }
}

void ObjectRecorder::CheckLengths()
{
    if(spin->points.size() != fpfh->points.size() || spin->points.size() != shot->points.size())
        throw RecordException("descriptor sizes are inconsistent");
}

QString ObjectRecorder::CreatePathAndFile(const QString dirName)
{
    if(g_frameIdx==1)
    {
        dstPath = ObjectReader::dsroot + QString("/") + dirName;

        QDir dir;
        if(!dir.exists(dstPath))
            if(!dir.mkdir(dstPath))
                throw RecordException("failed to create directory");
    }

    QString fileName = dstPath + QString("/") + QString("OBJ%1_%2_%3_%4.txt")
                                                .arg(ObjectReader::categoryIndex)
                                                .arg(ObjectReader::instanceIndex)
                                                .arg(ObjectReader::videoIndex)
                                                .arg(ObjectReader::frameIndex);
    return fileName;
}

void ObjectRecorder::RecordDescriptors(QString fileName)
{
    QFile file(fileName);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw RecordException(QString("cannot create descriptor file ")+fileName);
    QTextStream writer(&file);

    for(int i=0; i<indicesptr->size(); i++)
    {
        int idx = indicesptr->at(i);
        WriteDescriptor(writer, cwg[idx].s, DescSize);
        WriteDescriptor(writer, spin->at(i).histogram, SPIN_SIZE);
        WriteDescriptor(writer, fpfh->at(i).histogram, FPFHType::descriptorSize());
        WriteDescriptor(writer, shot->at(i).descriptor, SHOTType::descriptorSize());
        writer << '\n';
    }
    file.close();
}

void ObjectRecorder::WriteDescriptor(QTextStream& writer, const float* descriptor, const int size)
{
    for(int i=0; i<size; i++)
        writer << " " << descriptor[i];
}
