//seongwon
#ifndef PLANEEXTRACTOR_H
#define PLANEEXTRACTOR_H
#define NotPlane -10
#define Threshold 0.99

#include <QImage>
#include <QFile>
#include <vector>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "ClWork/cloperators.h"
#include "ClWork/clworker.h"
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
    void SetInputs(cl_float4* srcNormalCloud);
    void Work();

private:
    friend class PCWorker;

    int xy2num(int x, int y);
    void ExtractPlanes(cl_float4* normalCloud, vector<Plane>*& Planes, int* planeNum);
    void CompareNormal(int x, int y, cl_float4* normalCloud, int planeID, int* countPixel);

    vector<Plane>* planes;
    cl_float4* normalCloud;
    int* planemap;
    int viewOption;
    int planeNum;

    int count;
};

#endif // PLANEEXTRACTOR_H
