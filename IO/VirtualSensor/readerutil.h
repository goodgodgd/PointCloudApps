#ifndef READERUTIL_H
#define READERUTIL_H

#include <QFile>
#include "Share/project_common.h"
#include "Share/shared_types.h"
#include "attribtype.h"

class ReaderUtil
{
public:
    static MapNameData ReadAttributes(QTextStream& reader, int numProps)
    {
        MapNameData attributes;
        attributes.clear();
        int readCount=0;
        while(readCount < numProps)
        {
            if(reader.atEnd())
                throw QString("insufficient stream");

            QString line = reader.readLine();
            if(line.startsWith("["))
                throw QString("starting another object");
            if(line.startsWith("#"))
                continue;
            if(line.isEmpty())
                continue;

            QStringList words = line.split("=");
            if(words.size() != 2)
                throw QString("invalid line ") + line;

            QString attrName = words.at(0).trimmed();
            QString attrValue = words.at(1).trimmed();
            if(attrValue.startsWith("("))
                attributes.insert(PairNameData(attrName, AttribData(ToVector(attrValue))));
            else
                attributes.insert(PairNameData(attrName, AttribData(ToFloat(attrValue))));
            readCount++;
        }
        return attributes;
    }

    static void CheckIntegrity(const MapNameData& attribMap, const QStringList& attribList)
    {
        for(auto attrName : attribList)
        {
            if(attribMap.find(attrName)==attribMap.end())
                throw QString("No attribute: ") + QString(attrName);
        }
    }

private:
    static cl_float4 ToVector(QString& attrValue)
    {
        attrValue.remove("(");
        attrValue.remove(")");
        QStringList vectorStr = attrValue.split(",");
        if(vectorStr.size()!=3)
            throw QString("invalid vector") + attrValue;

        cl_float4 vector;
        bool bConvert=false;
        for(int i=0; i<3; i++)
        {
            vector.s[i] = vectorStr[i].trimmed().toFloat(&bConvert);
            if(bConvert==false)
                throw QString("invalid vector") + attrValue;
        }
        vector.s[3] = 0;
        return vector;
    }

    static cl_float ToFloat(const QString& attrValue)
    {
        bool bConvert=false;
        float value = attrValue.toFloat(&bConvert);
        if(bConvert==false)
            throw QString("invalid float") + attrValue;
        return value;
    }
};


#endif // READERUTIL_H
