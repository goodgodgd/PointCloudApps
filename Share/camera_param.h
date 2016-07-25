#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

//#define DEPTH_RANGE_MM  3500
//#define DEAD_RANGE_MM   100
//#define FOCAL_LENGTH    262.5f

#include "IO/FileReaders/rgbdposereader.h"

class CameraParam
{
    static constexpr int valid_range_beg_mm = 100;
    static constexpr int valid_range_end_mm = 3500;

    static constexpr float icl_fu = 525.f/2.f;
    static constexpr float icl_fv = 525.f/2.f;
    static constexpr float icl_cu = 319.5f/2.f;
    static constexpr float icl_cv = 239.5f/2.f;

    static constexpr float tum_fu = 525.f/2.f;
    static constexpr float tum_fv = 525.f/2.f;
    static constexpr float tum_cu = 319.5f/2.f;
    static constexpr float tum_cv = 239.5f/2.f;

public:
    static int DSID;
    static float flh()
    {
        if(DSID < 0 || DSID >= DSetID::DSetEnd)
            throw 0;
        else if(DSID < DSetID::TUM_freiburg1_desk)
            return icl_fu;
        else
            return tum_fu;
    }
    static float flv()
    {
        if(DSID < 0 || DSID >= DSetID::DSetEnd)
            throw 0;
        else if(DSID < DSetID::TUM_freiburg1_desk)
            return icl_fv;
        else
            return tum_fv;
    }
    static float cth()
    {
        if(DSID < 0 || DSID >= DSetID::DSetEnd)
            throw 0;
        else if(DSID < DSetID::TUM_freiburg1_desk)
            return icl_cu;
        else
            return tum_cu;
    }
    static float ctv()
    {
        if(DSID < 0 || DSID >= DSetID::DSetEnd)
            throw 0;
        else if(DSID < DSetID::TUM_freiburg1_desk)
            return icl_cv;
        else
            return tum_cv;
    }

    static int RangeBeg_mm() { return valid_range_beg_mm; }
    static int RangeEnd_mm() { return valid_range_end_mm; }
    static float RangeBeg_m() { return valid_range_beg_mm/1000.f; }
    static float RangeEnd_m() { return valid_range_end_mm/1000.f; }

};

#endif // CAMERA_PARAM_H
