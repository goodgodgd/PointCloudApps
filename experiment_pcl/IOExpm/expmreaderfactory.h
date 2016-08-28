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
        if(DSetID::ICL_NUIM_room1 <= DSID && DSID <= DSetID::ICL_NUIM_office1_noisy)
            return new ICLReader(DSID);
        else if(DSID == DSetID::Rgbd_Objects)
            return new ObjectReader();
        else
            return new TumReader(DSID);
    }
};

#endif // EXPMREADERFACTORY_H
