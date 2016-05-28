#ifndef SHAPEREADER_H
#define SHAPEREADER_H

#include "readerbase.h"
#include "ivirtualshape.h"
#include "virtualrectplane.h"
#include "virtualsphere.h"

typedef std::vector<IVirtualShape*> vecpShape;

class ShapeReader
{
public:
    ShapeReader() {}
    static vecpShape ReadShapes(QString filename)
    {
        QFile file(filename);
        if(file.open(QIODevice::ReadOnly)==false)
            throw QString("shape file not opened");
        QTextStream reader(&file);

        vecpShape shapes;

        while(!reader.atEnd())
        {
            QString line = reader.readLine();
            if(line.trimmed().compare("[rect]", Qt::CaseInsensitive)==0)
            {
                MapQStrFloat attributes = ReaderBase::ReadAttributes(reader, VirtualRectPlane::NUM_ATTRIB);
                shapes.push_back(new VirtualRectPlane(attributes));
            }
            else if(line.trimmed().compare("[sphere]", Qt::CaseInsensitive)==0)
            {
                MapQStrFloat attributes = ReaderBase::ReadAttributes(reader, VirtualSphere::NUM_ATTRIB);
                shapes.push_back(new VirtualSphere(attributes));
            }
        }
        return shapes;
    }
};


#endif // SHAPEREADER_H
