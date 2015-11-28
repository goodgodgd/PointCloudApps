#include "glvertexmanager.h"

QVector3D* gvm::r_posits = NULL;
QVector3D* gvm::r_colors = NULL;
QVector3D* gvm::r_normals = NULL;
GLfloat* gvm::r_ptsizes = NULL;

QVector3D* gvm::w_posits = NULL;
QVector3D* gvm::w_colors = NULL;
QVector3D* gvm::w_normals = NULL;
GLfloat* gvm::w_ptsizes = NULL;

const int gvm::totalsz = 80000;
const int gvm::ptbegin = 0;
const int gvm::lnbegin = 70000;
const int gvm::trbegin = 75000;
int gvm::ptnum = 0;
int gvm::lnnum = 0;
int gvm::trnum = 0;


GlVertexManager::GlVertexManager()
{
}

void GlVertexManager::InitVertices()
{
    // allocate read memory buffer
    r_posits = new QVector3D[totalsz];
    r_colors = new QVector3D[totalsz];
    r_normals = new QVector3D[totalsz];
    r_ptsizes = new GLfloat[totalsz];
    // allocate write memory buffer
    w_posits = new QVector3D[totalsz];
    w_colors = new QVector3D[totalsz];
    w_normals = new QVector3D[totalsz];
    w_ptsizes = new GLfloat[totalsz];

}

void GlVertexManager::AddVertex(eVertexType type, QVector3D& position, QVector3D& color, QVector3D& normal, int ptsize, bool b_complete)
{
    if(type==eVertexType::point)
    {
        w_posits[ptbegin + ptnum] = position;
        w_colors[ptbegin + ptnum] = color;
        w_normals[ptbegin + ptnum] = normal;
        w_ptsizes[ptbegin + ptnum] = ptsize;
        ptnum++;
    }
    else if(type==eVertexType::line)
    {
        if(b_complete == true && lnnum%2 != 1)
            return;
        w_posits[lnbegin + lnnum] = position;
        w_colors[lnbegin + lnnum] = color;
        w_normals[lnbegin + lnnum] = normal;
        w_ptsizes[lnbegin + lnnum] = ptsize;
        lnnum++;
    }
    else if(type==eVertexType::line)
    {
        if(b_complete == true && trnum%3 != 2)
            return;
        w_posits[trbegin + trnum] = position;
        w_colors[trbegin + trnum] = color;
        w_normals[trbegin + trnum] = normal;
        w_ptsizes[trbegin + trnum] = ptsize;
        trnum++;
    }
}

QVector3D* GlVertexManager::PositPtr()
{
    return r_posits;
}

QVector3D* GlVertexManager::ColorPtr()
{
    return r_colors;
}

QVector3D* GlVertexManager::NormalPtr()
{
    return r_normals;
}

GLfloat* GlVertexManager::PtsizePtr()
{
    return r_ptsizes;
}

void GlVertexManager::SwapRW()
{
    // swap read/write buffers
    QVector3D* tmpptr;
    tmpptr = r_posits;
    r_posits = w_posits;
    w_posits = tmpptr;

    tmpptr = r_colors;
    r_colors = w_colors;
    w_colors = tmpptr;

    tmpptr = r_normals;
    r_normals = w_normals;
    w_normals = tmpptr;

    GLfloat* tmpip;
    tmpip = r_ptsizes;
    r_ptsizes = w_ptsizes;
    w_ptsizes = tmpip;
}

