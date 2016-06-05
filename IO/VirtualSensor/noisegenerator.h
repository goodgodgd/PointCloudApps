#ifndef NOISEGENERATOR_H
#define NOISEGENERATOR_H

#include <cassert>
#include "Share/project_common.h"

class RandGenerator
{
protected:
    float param1, param2;
public:
    virtual float Generate() = 0;
    void Reset() { srand(0); }
};

class GaussianRandGenerator : public RandGenerator
{
    float mu() { return param1; }
    float sigma() { return param2; }

public:
    GaussianRandGenerator(float mu_, float sigma_)
//        : param1(mu_), param2(sigma_)
    {
        param1 = mu_;
        param2 = sigma_;
    }

    virtual float Generate()
    {
        static float X1, X2;
        static int call = 0;
        float U1, U2, W, mult;

        if(call == 1)
        {
            call = !call;
            return (mu() + sigma() * X2);
        }

        do
        {
            U1 = -1 + ((float) rand () / RAND_MAX) * 2;
            U2 = -1 + ((float) rand () / RAND_MAX) * 2;
            W = powf(U1, 2) + powf(U2, 2);
        }
        while (W >= 1 || W == 0);

        mult = sqrtf((-2 * logf(W)) / W);
        X1 = U1 * mult;
        X2 = U2 * mult;

        call = !call;

        return (mu() + sigma()*X1);
    }
};

class UniformRandGenerator : public RandGenerator
{
    float low() { return param1; }
    float width() { return param2; }

public:
    UniformRandGenerator(float low_, float high_)
//        : param1(low_), param2(high_)
    {
        param1 = low_;
        param2 = high_;
    }

    virtual float Generate()
    {
        return (low() + ((float)rand()/RAND_MAX)*width());
    }
};

#endif // NOISEGENERATOR_H
