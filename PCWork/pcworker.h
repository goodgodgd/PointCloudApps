#ifndef PCWORKER_H
#define PCWORKER_H

#include <QImage>
#include <QElapsedTimer>
#include "project_common.h"
#include "ClWork/clworker.h"
#include "IO/glvertexmanager.h"
#include "ClWork/cloperators.h"
#include "Share/sharedenums.h"

class PCWorker
{
public:
    PCWorker();
    ~PCWorker();
    void SetInputs(QImage& srcColorImg, cl_float4* srcPointCloud, int inViewOption);
    void Work();

private:
    void DrawPointCloud(cl_float4* pointCloud, cl_float4* normalCloud, int viewOption);

    cl_float4*      pointCloud;
    cl_float4*      normalCloud;
    DescType*       descriptorCloud;
    QImage          colorImg;
    int             viewOption;

    CLWorker*       clworker;
    QElapsedTimer   eltimer;
};

#endif // PCWORKER_H
