#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QElapsedTimer>
#include "project_common.h"
#include "ClWork/clworker.h"
#include "Shared/operators.h"

#define VIZ3D

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void SetInputs(QImage& colorImg, cl_float4* srcPointCloud);
    void Work();

    cl_float4* m_pointCloud;
    cl_float4* m_normalCloud;
#ifdef VIZ3D
    QVector3D** m_qvPointCloud;
    QImage m_colorImg;
#endif // VIZ3D

    CLWorker* m_clworker;
    QElapsedTimer m_eltimer;
};

#endif // PCWORKER_H
