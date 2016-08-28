#ifndef OBJECTRECORDER_H
#define OBJECTRECORDER_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/gpu/features/features.hpp>
#include <QDir>
#include <QTextStream>
#include <iostream>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "Share/shared_enums.h"
#include "IOExpm/objectreader.h"
#include "ShareExpm/expm_common.h"
#include "ShareExpm/expm_structs.h"

class ObjectRecorder
{
public:
    ObjectRecorder();
    void Record(boost::shared_ptr<std::vector<int>> indicesptr_
                , const DescType* pcwg_
                , pcl::PointCloud<SpinImageType>::Ptr spin_
                , pcl::PointCloud<FPFHType>::Ptr fpfh_
                , pcl::PointCloud<SHOTType>::Ptr shot_
                , pcl::PointCloud<TrisiType>::Ptr trisi_
                );

private:
    void CheckLengths();
    QString CreatePathAndFile();
    void RecordDescriptors(QString filePath);
    void WriteDescriptor(QTextStream& writer, const float* descriptor, const int size);

    QString dstPath;
    QString objFileName;
    boost::shared_ptr<std::vector<int>> indicesptr;
    const DescType* pcwg;
    pcl::PointCloud<SpinImageType>::Ptr spin;
    pcl::PointCloud<FPFHType>::Ptr fpfh;
    pcl::PointCloud<SHOTType>::Ptr shot;
    pcl::PointCloud<NarfType>::Ptr narf;
    pcl::PointCloud<TrisiType>::Ptr trisi;
};

#endif // OBJECTRECORDER_H
