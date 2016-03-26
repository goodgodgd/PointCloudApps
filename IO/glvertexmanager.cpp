#include "glvertexmanager.h"

const int gvm::totalsz = 90000;
const int gvm::ptbegin = 0;
const int gvm::lnbegin = 70000;
const int gvm::trbegin = 85000;

QVector3D* gvm::r_posits = NULL;
QVector3D* gvm::r_colors = NULL;
QVector3D* gvm::r_normals = NULL;
GLfloat* gvm::r_ptsizes = NULL;
int gvm::r_ptnum = 0;
int gvm::r_lnnum = 0;
int gvm::r_trnum = 0;

QVector3D* gvm::w_posits = NULL;
QVector3D* gvm::w_colors = NULL;
QVector3D* gvm::w_normals = NULL;
GLfloat* gvm::w_ptsizes = NULL;
int gvm::w_ptnum = 0;
int gvm::w_lnnum = 0;
int gvm::w_trnum = 0;


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
    if(type==VertexType::point)
    {
        if(w_ptnum >= lnbegin)
            return;
        w_posits[ptbegin + w_ptnum] = position;
        w_colors[ptbegin + w_ptnum] = color;
        w_normals[ptbegin + w_ptnum] = normal;
        w_ptsizes[ptbegin + w_ptnum] = ptsize;
        w_ptnum++;
    }
    else if(type==VertexType::line)
    {
        if(lnbegin + w_lnnum >= trbegin)
            return;
        if(b_complete == true && w_lnnum%2 != 1)
            return;
        w_posits[lnbegin + w_lnnum] = position;
        w_colors[lnbegin + w_lnnum] = color;
        w_normals[lnbegin + w_lnnum] = normal;
        w_ptsizes[lnbegin + w_lnnum] = ptsize;
        w_lnnum++;
    }
    else if(type==VertexType::triangle)
    {
        if(trbegin + w_trnum >= totalsz)
            return;
        if(b_complete == true && w_trnum%3 != 2)
            return;
        w_posits[trbegin + w_trnum] = position;
        w_colors[trbegin + w_trnum] = color;
        w_normals[trbegin + w_trnum] = normal;
        w_ptsizes[trbegin + w_trnum] = ptsize;
        w_trnum++;
    }
}

void GlVertexManager::AddVertex(eVertexType type, cl_float4& position, cl_float4& color, cl_float4& normal, int ptsize, bool b_complete)
{
    if(type==VertexType::point)
    {
        if(w_ptnum >= lnbegin)
            return;
        w_posits[ptbegin + w_ptnum] << position;
        w_colors[ptbegin + w_ptnum] << color;
        w_normals[ptbegin + w_ptnum] << normal;
        w_ptsizes[ptbegin + w_ptnum] = ptsize;
        w_ptnum++;
    }
    else if(type==VertexType::line)
    {
        if(lnbegin + w_lnnum >= trbegin)
            return;
        if(b_complete == true && w_lnnum%2 == 0)
            return;
        if(b_complete == false && w_lnnum%2 == 1)
            return;
        w_posits[lnbegin + w_lnnum] << position;
        w_colors[lnbegin + w_lnnum] << color;
        w_normals[lnbegin + w_lnnum] << normal;
        w_ptsizes[lnbegin + w_lnnum] = ptsize;
        w_lnnum++;
    }
    else if(type==VertexType::triangle)
    {
        if(trbegin + w_trnum >= totalsz)
            return;
        if(b_complete == true && w_trnum%3 != 2)
            return;
        w_posits[trbegin + w_trnum] << position;
        w_colors[trbegin + w_trnum] << color;
        w_normals[trbegin + w_trnum] << normal;
        w_ptsizes[trbegin + w_trnum] = ptsize;
        w_trnum++;
    }
}

void GlVertexManager::ShowAddedVertices()
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

    r_ptnum = w_ptnum;
    w_ptnum = 0;
    r_lnnum = w_lnnum;
    w_lnnum = 0;
    r_trnum = w_trnum;
    w_trnum = 0;
}

void GlVertexManager::AddCartesianAxes()
{
    /// draw example points
    QVector3D pvpos;    // point vertex position
    QVector3D vcol;     // vertex color
    // point normal vector
    QVector3D vnml = QVector3D(1,1,1);  // vertex normal
    vnml.normalize();

    /// draw coordinate axes
    QVector3D lnpos1;   // line end point1 vertex position at origin
    QVector3D lnpos2;   // line end point2 vertex position

    // draw X-axis with red line
    vcol = QVector3D(1,0,0);    // red
    // add line vertex 1 at origin
    lnpos1 = QVector3D(0,0,0);
    AddVertex(VertexType::line, lnpos1, vcol, vnml, 1);
    // add line vertex 2 along X-axis
    lnpos2 = QVector3D(0.2f,0,0);
    AddVertex(VertexType::line, lnpos2, vcol, vnml, 1, true); // when last vertex of line is added, last argument must be "true"

    // draw Y-axis with green line
    vcol = QVector3D(0,1,0);    // green
    // add line vertex 1 at origin
    lnpos1 = QVector3D(0,0,0);
    AddVertex(VertexType::line, lnpos1, vcol, vnml, 1);
    // add line vertex 2 along Y-axis
    lnpos2 = QVector3D(0,0.2f,0);
    AddVertex(VertexType::line, lnpos2, vcol, vnml, 1, true); // when last vertex of line is added, last argument must be "true"

    // draw Z-axis with blue line
    vcol = QVector3D(0,0,1);    // blue
    // add line vertex 1 at origin
    lnpos1 = QVector3D(0,0,0);
    AddVertex(VertexType::line, lnpos1, vcol, vnml, 1);
    // add line vertex 2 along Z-axis
    lnpos2 = QVector3D(0,0,0.2f);
    AddVertex(VertexType::line, lnpos2, vcol, vnml, 1, true); // when last vertex of line is added, last argument must be "true"
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

int GlVertexManager::PtBegin()
{
    return ptbegin;
}

int GlVertexManager::LnBegin()
{
    return lnbegin;
}

int GlVertexManager::TrBegin()
{
    return trbegin;
}

int GlVertexManager::PtNum()
{
    return r_ptnum;
}

int GlVertexManager::LnNum()
{
    return r_lnnum;
}

int GlVertexManager::TrNum()
{
    return r_trnum;
}
