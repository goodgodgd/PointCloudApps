//seongwon
#ifndef PLANEEXTRACTOR_H
#define PLANEEXTRACTOR_H
#define NotPlane -10
#define Threshold 0.9965

#include <QImage>
#include <QFile>
#include <vector>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "ClUtils/cloperators.h"
#include "Share/sharedenums.h"
using namespace std;

struct Plane
{
    Plane(int _ID, int _xl, int _xh, int _yl, int _yh, int _numpts)
    {
        ID = _ID;
        xl = _xl;
        xh = _xh;
        yl = _yl;
        yh = _yh;
        numpts = _numpts;
    }
    int ID, xl, xh, yl, yh, numpts;
};

class PlaneExtractor
{
public:
    PlaneExtractor();
    ~PlaneExtractor();
    void ExtractPlanes(cl_float4* normalCloud);
    int planeNum;
    int* planemap;

private:

    inline int xy(int x, int y);

    void CompareNormal(int x, int y, cl_float4* normalCloud, int planeID, int* countPixel);

    vector<Plane>* planes;

    int smalls[50000];
    int smalls_num;
    int count;
};

#endif // PLANEEXTRACTOR_H
