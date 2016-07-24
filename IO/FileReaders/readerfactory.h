#ifndef READERFACTORY_H
#define READERFACTORY_H

#include "iclreader.h"
#include "tumreader.h"

class ReaderFactory
{
public:
    ReaderFactory();
    static RgbdPoseReader* GetInstance(const int DSID)
    {
        if(DSID < DSetID::TUM_freiburg1_desk)
            return new ICLReader(DSID);
        else
            return new TumReader(DSID);
    }
};

#endif // READERFACTORY_H
