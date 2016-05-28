#ifndef READERBASE_H
#define READERBASE_H

#include <QFile>
#include "Share/project_common.h"
#include "Share/shared_types.h"

class ReaderUtil
{
public:
    static MapQStrFloat ReadAttributes(QTextStream& reader, int numProps)
    {
        MapQStrFloat attributes;
        attributes.clear();
        bool bConvert=false;
        for(int i=0; i<numProps; ++i)
        {
            if(reader.atEnd())
                throw QString("insufficient stream");

            QString line = reader.readLine();
            QStringList words = line.split("=");
            if(words.size() != 2)
                throw QString("invalid line");

            QString attrName = words.at(0).trimmed();
            float value = words.at(1).trimmed().toFloat(&bConvert);
            if(bConvert==false)
                throw QString("invalid number");

            attributes.insert(PairQStrFloat(attrName, value));
        }
        return attributes;
    }

    static void CheckIntegrity(const MapQStrFloat& attribMap, const QStringList& attribList)
    {
        for(auto attrName : attribList)
        {
            if(attribMap.find(attrName)==attribMap.end())
                throw QString("No attribute: ") + QString(attrName);
        }
    }
};


#endif // READERBASE_H
