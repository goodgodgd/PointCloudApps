#ifndef READERFACTORY_H
#define READERFACTORY_H

#include "IO/FileReaders/objectreader.h"
#include "IO/FileReaders/rgbdasyncreader.h"
#include "IO/FileReaders/rgbdsyncreader.h"
#include "IO/FileReaders/samplereader.h"

class ReaderFactory
{
public:
    ReaderFactory();
    static RgbdReaderInterface* GetInstance(QString localDataPath, bool readSample=false)
    {
        if(readSample)
            return new SampleReader(localDataPath);
        else if(localDataPath.contains("CoRBS"))
            return new RgbdAsyncReader(localDataPath);
        else if(localDataPath.contains("rgbd-scene"))
            return new RgbdSyncReader(localDataPath);
        else
            return new ObjectReader;
    }
};

#endif // READERFACTORY_H
