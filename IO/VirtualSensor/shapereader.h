#ifndef SHAPEREADER_H
#define SHAPEREADER_H

#include "readerutil.h"
#include "IO/VirtualShape/ivirtualshape.h"
#include "IO/VirtualShape/virtualrectplane.h"
#include "IO/VirtualShape/virtualsphere.h"
#include "IO/VirtualShape/virtualcuboid.h"

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
            if(line.startsWith("##"))
                break;
            if(line.trimmed().compare("[rect]", Qt::CaseInsensitive)==0)
            {
                MapNameData attributes = ReaderUtil::ReadAttributes(reader, VirtualRectPlane::NUM_ATTRIB);
                shapes.push_back(new VirtualRectPlane(attributes));
            }
            else if(line.trimmed().compare("[sphere]", Qt::CaseInsensitive)==0)
            {
                MapNameData attributes = ReaderUtil::ReadAttributes(reader, VirtualSphere::NUM_ATTRIB);
                shapes.push_back(new VirtualSphere(attributes));
            }
            else if(line.trimmed().compare("[cuboid]", Qt::CaseInsensitive)==0)
            {
                MapNameData attributes = ReaderUtil::ReadAttributes(reader, VirtualCuboid::NUM_ATTRIB);
                shapes.push_back(new VirtualCuboid(attributes));
            }
        }
        return shapes;
    }
};


#endif // SHAPEREADER_H
