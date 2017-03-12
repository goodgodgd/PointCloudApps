#ifndef EXPMREADERFACTORY_H
#define EXPMREADERFACTORY_H

#include "IO/FileReaders/objectreader.h"
#include "IO/FileReaders/rgbdasyncreader.h"
#include "IO/FileReaders/depthreader.h"

class ExpmReaderFactory
{
public:
    ExpmReaderFactory();
    static RgbdReaderInterface* GetInstance(QString localDataPath)
    {
        if(localDataPath.contains("CoRBS"))
            return new RgbdAsyncReader(localDataPath);
        else if(localDataPath.contains("rgbd-scene"))
            return new RgbdAsyncReader(localDataPath);
        else
            return new ObjectReader();
    }
};

#endif // EXPMREADERFACTORY_H
