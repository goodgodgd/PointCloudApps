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
        if(DSID < DSetID::TUM_freiburg1_desk)
            return new ICLReader(DSID);
        else if(DSID < DSetID::Rgbd_Objects)
            return new TumReader(DSID);
        else
            return nullptr;
    }
};

#endif // READERFACTORY_H
