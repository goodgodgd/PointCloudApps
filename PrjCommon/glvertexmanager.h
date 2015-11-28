#ifndef GLVERTEXMANAGER_H
#define GLVERTEXMANAGER_H

#include <QGLWidget>
#include <QVector3D>

enum eVertexType
{
    point,
    line,
    triangle
};

class GlVertexManager
{
public:
    GlVertexManager();
    static void InitVertices();
    static void AddVertex(eVertexType type, QVector3D& position, QVector3D& color, QVector3D& normal, int ptsize, bool b_complete=false);
    static QVector3D* PositPtr();
    static QVector3D* NormalPtr();
    static QVector3D* ColorPtr();
    static GLfloat* PtsizePtr();
    static void SwapRW();

    static const int totalsz;
    static const int ptbegin;
    static const int lnbegin;
    static const int trbegin;
    static int ptnum;
    static int lnnum;
    static int trnum;

private:

    static QVector3D* r_posits;
    static QVector3D* r_colors;
    static QVector3D* r_normals;
    static GLfloat* r_ptsizes;

    static QVector3D* w_posits;
    static QVector3D* w_colors;
    static QVector3D* w_normals;
    static GLfloat* w_ptsizes;
};

typedef GlVertexManager gvm;

#endif // GLVERTEXMANAGER_H
