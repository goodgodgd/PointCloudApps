#ifndef SAMPLEREADER_H
#define SAMPLEREADER_H

#include <stdlib.h>
#include <utility>
#include "IO/FileReaders/rgbdposereader.h"

class SampleReader : public RgbdPoseReader
{
public:
    SampleReader(const QString localPath);
    virtual ~SampleReader() {}
    virtual void ReadRgbdFrame(const int index, QImage& color, QImage& depth);
    virtual std::vector<int> GetSamplePixel(const int index);
    virtual int GetLength() { return depthList.size(); }

protected:
    virtual void LoadInitInfo();
    virtual QString ColorName(const int index);
    virtual QString DepthName(const int index);

    QStringList depthList;
    std::vector<cl_int2> pixels;
    std::vector<cl_float4> points;
};

#endif // SAMPLEREADER_H
