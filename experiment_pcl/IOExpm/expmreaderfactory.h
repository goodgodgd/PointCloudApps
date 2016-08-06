#ifndef EXPMREADERFACTORY_H
#define EXPMREADERFACTORY_H

#include "IO/FileReaders/iclreader.h"
#include "IO/FileReaders/tumreader.h"
#include "objectreader.h"

class ExpmReaderFactory
{
public:
    ExpmReaderFactory();
    static RgbdReaderInterface* GetInstance(const int DSID)
    {
        if(DSID < DSetID::TUM_freiburg1_desk)
            return new ICLReader(DSID);
        else if(DSID < DSetID::Rgbd_Objects)
            return new TumReader(DSID);
        else
            return new ObjectReader();
    }
};

#endif // EXPMREADERFACTORY_H
