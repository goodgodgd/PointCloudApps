#ifndef READERFACTORY_H
#define READERFACTORY_H

#include "iclreader.h"
#include "tumreader.h"

class ReaderFactory
{
public:
    ReaderFactory();
    static RgbdReaderInterface* GetInstance(const int DSID)
    {
        if(DSetID::ICL_NUIM_room1 <= DSID && DSID <= DSetID::ICL_NUIM_office1_noisy)
            return new ICLReader(DSID);
        else if(DSID < DSetID::Rgbd_Objects)
            return new TumReader(DSID);
        else
            return nullptr;
    }
};

#endif // READERFACTORY_H
