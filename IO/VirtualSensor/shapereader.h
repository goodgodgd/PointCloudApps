#ifndef SHAPEREADER_H
#define SHAPEREADER_H

#include "readerutil.h"
#include "IO/VirtualShape/ivirtualshape.h"
#include "IO/VirtualShape/virtualrectplane.h"
#include "IO/VirtualShape/virtualsphere.h"
#include "IO/VirtualShape/virtualcuboid.h"
#include "IO/VirtualShape/virtualcylinder.h"
#include "IO/VirtualShape/virtualellipsoid.h"

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

            MapNameData attributes = ReaderUtil::ReadAttributes(reader);

            if(line.trimmed().compare("[rect]", Qt::CaseInsensitive)==0)
                shapes.push_back(new VirtualRectPlane(attributes));
            else if(line.trimmed().compare("[cuboid]", Qt::CaseInsensitive)==0)
                shapes.push_back(new VirtualCuboid(attributes));
            else if(line.trimmed().compare("[sphere]", Qt::CaseInsensitive)==0)
                shapes.push_back(new VirtualSphere(attributes));
            else if(line.trimmed().compare("[cylinder]", Qt::CaseInsensitive)==0)
                shapes.push_back(new VirtualCylinder(attributes));
            else if(line.trimmed().compare("[ellipsoid]", Qt::CaseInsensitive)==0)
                shapes.push_back(new VirtualEllipsoid(attributes));
        }
        return shapes;
    }
};


#endif // SHAPEREADER_H
