#ifndef GLVERTEXMANAGER_H
#define GLVERTEXMANAGER_H

#include <QGLWidget>
#include <QVector3D>
#include "project_common.h"
#include "ClWork/cloperators.h"

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
    static void AddVertex(eVertexType type, cl_float4& position, cl_float4& color, cl_float4& normal, int ptsize, bool b_complete=false);
    static void SwapRW();

    static QVector3D* PositPtr();
    static QVector3D* NormalPtr();
    static QVector3D* ColorPtr();
    static GLfloat* PtsizePtr();

    static int PtBegin();
    static int LnBegin();
    static int TrBegin();
    static int PtNum();
    static int LnNum();
    static int TrNum();

private:

    static QVector3D* r_posits;
    static QVector3D* r_colors;
    static QVector3D* r_normals;
    static GLfloat* r_ptsizes;

    static QVector3D* w_posits;
    static QVector3D* w_colors;
    static QVector3D* w_normals;
    static GLfloat* w_ptsizes;

    static const int totalsz;
    static const int ptbegin;
    static const int lnbegin;
    static const int trbegin;
    static int w_ptnum;
    static int w_lnnum;
    static int w_trnum;
    static int r_ptnum;
    static int r_lnnum;
    static int r_trnum;
};

typedef GlVertexManager gvm;

#endif // GLVERTEXMANAGER_H
