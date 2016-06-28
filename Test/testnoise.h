#ifndef TESTNOISE_H
#define TESTNOISE_H

#include "IO/VirtualSensor/noisegenerator.h"

namespace Test
{

inline void TestGaussianRand()
{
#define NUMRAND     1000
#define NUMBINS     11
#define ROUND(a)    ((a>=0) ? (int)(a+0.5f) : (int)(a-0.5f))

    const float mu = 1.f;
    const float sigma = 0.1f;
    const int half = NUMBINS/2;
    int histogram[NUMBINS];
    float normRand;
    int index;
    memset(histogram, 0x00, sizeof(histogram));

    GaussianRandGenerator grg(mu, sigma);

    for(int i=0; i<NUMRAND; i++)
    {
        normRand = (grg.Generate()-mu)/sigma;
        index = ROUND(normRand) + half;
//        if(i<20)
//            qDebug() << normRand << (ROUND(normRand) << index << half;
        if(index<0 || index>=NUMBINS)
            continue;
        histogram[index]++;
    }

    {
        QDebug dbg = qDebug();
        dbg << "gaussian rand test";
        for(int i=0; i<NUMBINS; i++)
            dbg << histogram[i];
    }

    for(int i=1; i<=half; i++)
    {
        if(histogram[half + i-1] > NUMRAND/20)
            assert(histogram[half + i-1] > histogram[half + i]);
        if(histogram[half - i+1] > NUMRAND/20)
            assert(histogram[half - i+1] > histogram[half - i]);
    }
}

inline void TestUniformRand()
{
#define NUMRAND     10000
#define NUMBINS     11
#define ROUND(a)    (a>=0) ? (int)(a+0.5f) : (int)(a-0.5f)

    const float low = 10.f;
    const float high = 20.f;
    int histogram[NUMBINS];
    float normRand;
    int index;
    memset(histogram, 0x00, sizeof(histogram));

    UniformRandGenerator urg(low, high);

    for(int i=0; i<NUMRAND; i++)
    {
        normRand = (urg.Generate()-low)/(high-low)*(float)NUMBINS;
        index = (int)(normRand);
        if(index<0 || index>=NUMBINS)
            continue;
        histogram[index]++;
    }

    {
        QDebug dbg = qDebug();
        dbg << "uniform rand test";
        for(int i=0; i<NUMBINS; i++)
            dbg << histogram[i];
    }

    for(int i=1; i<NUMBINS; i++)
        assert(abs(histogram[0]-histogram[i]) < NUMRAND/50);
}

}
#endif // TESTNOISE_H
