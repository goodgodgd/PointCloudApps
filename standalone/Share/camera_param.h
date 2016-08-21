#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include "Share/exceptions.h"
#include "IO/FileReaders/rgbdposereader.h"

namespace CameraType
{
enum Enum
{
    SCENE_ICL,
    SCENE_TUM,
    OBJECT,
};
}

class CameraParam
{
    static constexpr int valid_range_beg_mm = 100;
    static constexpr int valid_range_end_mm = 3500;

    static constexpr float icl_flh = 481.2f/2.f;
    static constexpr float icl_flv = 480.f/2.f;
    static constexpr float icl_cth = 319.5f/2.f;
    static constexpr float icl_ctv = 239.5f/2.f;

    static constexpr float tum_flh = 525.f/2.f;
    static constexpr float tum_flv = 525.f/2.f;
    static constexpr float tum_cth = 319.5f/2.f;
    static constexpr float tum_ctv = 239.5f/2.f;

    static constexpr float obj_flh = 570.3f;
    static constexpr float obj_flv = 570.3f;
    static constexpr float obj_cth = 320.f/2.f;
    static constexpr float obj_ctv = 240.f/2.f;

    static constexpr int sample_range_clean = 3.0f;
    static constexpr int track_range_clean = 3.5f;
    static constexpr int sample_range_noisy = 1.2f;
    static constexpr int track_range_noisy = 1.5f;

public:
    static int dsetType;
    static float flh()
    {
        if(dsetType<=DSetID::ICL_NUIM_room1_noisy)
            return icl_flh;
        else if(dsetType<=DSetID::TUM_freiburg3_long)
            return tum_flh;
        else if(dsetType==DSetID::Rgbd_Objects)
            return obj_flh;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float flv()
    {
        if(dsetType<=DSetID::ICL_NUIM_room1_noisy)
            return icl_flv;
        else if(dsetType<=DSetID::TUM_freiburg3_long)
            return tum_flv;
        else if(dsetType==DSetID::Rgbd_Objects)
            return obj_flv;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float cth()
    {
        if(dsetType<=DSetID::ICL_NUIM_room1_noisy)
            return icl_cth;
        else if(dsetType<=DSetID::TUM_freiburg3_long)
            return tum_cth;
        else if(dsetType==DSetID::Rgbd_Objects)
            return obj_cth;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float ctv()
    {
        if(dsetType<=DSetID::ICL_NUIM_room1_noisy)
            return icl_ctv;
        else if(dsetType<=DSetID::TUM_freiburg3_long)
            return tum_ctv;
        else if(dsetType==DSetID::Rgbd_Objects)
            return obj_ctv;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float sampleRange()
    {
        if(dsetType==DSetID::ICL_NUIM_office1 || dsetType==DSetID::ICL_NUIM_room1)
            return sample_range_clean;
        else
            return sample_range_noisy;
    }
    static float trackRange()
    {
        if(dsetType==DSetID::ICL_NUIM_office1 || dsetType==DSetID::ICL_NUIM_room1)
            return track_range_clean;
        else
            return track_range_noisy;
    }

    static int RangeBeg_mm() { return valid_range_beg_mm; }
    static int RangeEnd_mm() { return valid_range_end_mm; }
    static float RangeBeg_m() { return valid_range_beg_mm/1000.f; }
    static float RangeEnd_m() { return valid_range_end_mm/1000.f; }

};

#endif // CAMERA_PARAM_H
