#include "objectrecorder.h"

ObjectRecorder::ObjectRecorder()
{
}

void ObjectRecorder::Record(boost::shared_ptr<std::vector<int>> indicesptr_
                                , const DescType* pcwg_
                                , pcl::PointCloud<SpinImageType>::Ptr spin_
                                , pcl::PointCloud<FPFHType>::Ptr fpfh_
                                , pcl::PointCloud<SHOTType>::Ptr shot_
                                , pcl::PointCloud<TrisiType>::Ptr trisi_
                                )
{
    indicesptr = indicesptr_;
    pcwg = pcwg_;
    spin = spin_;
    fpfh = fpfh_;
    shot = shot_;
    trisi = trisi_;
//    narf = narf_;

    try {
        CheckLengths();
        QString filePath = CreatePathAndFile();
        RecordDescriptors(filePath);
        qDebug() << "record completed" << filePath;
    }
    catch (RecordException exception) {
        qDebug() << "RecordException:" << exception.msg;
    }
}

void ObjectRecorder::CheckLengths()
{
    if(fpfh->points.size() != shot->points.size() || fpfh->points.size() != trisi->points.size())
        throw RecordException("descriptor sizes are inconsistent");
}

QString ObjectRecorder::CreatePathAndFile()
{
    if(dstPath.isEmpty())
    {
        int radius = (int)(DESC_RADIUS*100.f);
        dstPath = ObjectReader::dsroot + QString("/_DescR%1").arg(radius);

        QDir dir;
        if(!dir.exists(dstPath))
            if(!dir.mkdir(dstPath))
                throw RecordException("failed to create directory");
    }

    objFileName = QString("OBJ_") + ObjectReader::objectID + QString(".txt");
    QString filePath = dstPath + QString("/") + objFileName;
    return filePath;
}

void ObjectRecorder::RecordDescriptors(QString filePath)
{
    QFile file(filePath);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)==false)
        throw RecordException(QString("cannot create descriptor file ")+filePath);
    QTextStream writer(&file);

    for(int i=0; i<indicesptr->size(); i++)
    {
        int idx = indicesptr->at(i);
        WriteDescriptor(writer, pcwg[idx].s, DescSize);
//        WriteDescriptor(writer, spin->at(i).histogram, SpinImageType::descriptorSize());
        WriteDescriptor(writer, fpfh->at(i).histogram, FPFHType::descriptorSize());
        WriteDescriptor(writer, shot->at(i).descriptor, SHOTType::descriptorSize());
        WriteDescriptor(writer, trisi->at(i).histogram, TrisiType::descriptorSize());
        writer << '\n';
    }
    file.close();
}

void ObjectRecorder::WriteDescriptor(QTextStream& writer, const float* descriptor, const int size)
{
    for(int i=0; i<size; i++)
        writer << " " << descriptor[i];
}
