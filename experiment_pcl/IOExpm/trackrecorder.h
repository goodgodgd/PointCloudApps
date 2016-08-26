#ifndef DESCRIPTORRECORDER_H
#define DESCRIPTORRECORDER_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/gpu/features/features.hpp>
#include <QDir>
#include <QTextStream>
#include <iostream>
#include "Share/project_common.h"
#include "Share/fordescriptor.h"
#include "ShareExpm/expm_common.h"
#include "ShareExpm/expm_structs.h"
#include "IO/FileReaders/rgbdposereader.h"

class TrackRecorder
{
public:
    TrackRecorder();
    void Record(const std::vector<TrackPoint>* trackPoints_
                , const DescType* pcwg_
                , pcl::PointCloud<SpinImageType>::Ptr spin_
                , pcl::PointCloud<FPFHType>::Ptr fpfh_
                , pcl::PointCloud<SHOTType>::Ptr shot_
                , pcl::PointCloud<TrisiType>::Ptr trisi_
                );

private:
    void CheckLengths();
    QString CreatePathAndFile(const QString dirName, const QString fileName);
    void RecordDescriptors(QString fileName);
    void WriteTrackInfo(QTextStream& writer, const TrackPoint trackPoint);
    void WriteDescriptor(QTextStream& writer, const float* descriptor, const int size);

    QString dstPath;
    const std::vector<TrackPoint>* trackPoints;
    const DescType* pcwg;
    pcl::PointCloud<SpinImageType>::Ptr spin;
    pcl::PointCloud<FPFHType>::Ptr fpfh;
    pcl::PointCloud<SHOTType>::Ptr shot;
    pcl::PointCloud<NarfType>::Ptr narf;
    pcl::PointCloud<TrisiType>::Ptr trisi;
};

#endif // DESCRIPTORRECORDER_H
