#ifndef NOISEREADER_H
#define NOISEREADER_H

#include "Share/project_common.h"
#include "readerutil.h"
#include "noisegenerator.h"

class NoiseReader
{
    enum Enum
    {
        NOTFOUND,
        GAUSSIAN,
        UNIFORM
    };

public:
    NoiseReader() {}
    static RandGenerator* ReadNoiseGenerator(const QString filename)
    {
        QFile file(filename);
        if(file.open(QIODevice::ReadOnly)==false)
            throw QString("noise file not opened");
        QTextStream reader(&file);
        int noiseType = NOTFOUND;
        while(!reader.atEnd())
        {
            QString line = reader.readLine();
            if(line.startsWith("##"))
                break;
            if(line.trimmed().compare("[gaussian_noise]", Qt::CaseInsensitive)==0)
            {
                noiseType = GAUSSIAN;
                break;
            }
            else if(line.trimmed().compare("[uniform_noise]", Qt::CaseInsensitive)==0)
            {
                noiseType = UNIFORM;
                break;
            }
        }
        if(noiseType == NOTFOUND)
            throw QString("cannot find noise params");

        MapNameData attribMap = ReaderUtil::ReadAttributes(reader);

        QStringList attribList;
        attribList << "param1" << "param2";
        ReaderUtil::CheckIntegrity(attribMap, attribList);

        float param1, param2;
        param1 = attribMap[attribList[0]].GetValue();
        param2 = attribMap[attribList[1]].GetValue();
        qDebug() << "noise param1" << param1 << "param2" << param2;

        RandGenerator* randGen = nullptr;
        if(noiseType == GAUSSIAN)
            randGen = new GaussianRandGenerator(param1, param2);
        else if(noiseType == UNIFORM)
            randGen = new UniformRandGenerator(param1, param2);
        else
            throw QString("noise type is not specified");
        return randGen;
    }
};


#endif // NOISEREADER_H
