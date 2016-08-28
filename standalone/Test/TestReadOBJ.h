#ifndef TESTREADOBJ_H
#define TESTREADOBJ_H

#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <IO/glvertexmanager.h>

namespace Test
{

inline void TestReadOBJ()
{
    QFile file("../Mesh_H.obj");
    file.open(QIODevice::ReadOnly|QIODevice::Text);
    QTextStream reader(&file);
    QString line;
    QStringList list;
    int vtxcount = 0;
    int texcount = 0;
    int norcount = 0;
    int facecount = 0;
    cl_float4 vertex;
    cl_float4 color = (cl_float4){0.5f, 0.5f, 0.5f};
    cl_float4 normal = (cl_float4){0, 0, 1.f};

    while(!reader.atEnd())
    {
        line = reader.readLine();
        if(line.startsWith("v "))
        {
            ++vtxcount;
            if(vtxcount < 100)
                qDebug() << "vertex" << line;
            list = line.split(" ");
            vertex = (cl_float4){list.at(1).toFloat(), list.at(2).toFloat(), list.at(3).toFloat(), 0};
            gvm::AddVertex(VertexType::point, vertex, color, normal, 2);
        }
        else if(line.startsWith("vt"))
        {
            ++texcount;
            if(texcount < 100)
                qDebug() << "texture" << line;
        }
        else if(line.startsWith("vn"))
        {
            ++norcount;
            if(norcount < 100)
                qDebug() << "normal" << line;
        }
        else if(line.startsWith("f"))
        {
            ++facecount;
            if(facecount < 100)
                qDebug() << "face" << line;
        }
    }
    qDebug() << "total vertices" << vtxcount << texcount << norcount << facecount;

    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();
}

}
#endif // TESTREADOBJ_H
