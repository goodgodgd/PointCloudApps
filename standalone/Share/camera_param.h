#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include "Share/exceptions.h"
#include "IO/FileReaders/rgbdposereader.h"

namespace CameraType
{
enum Enum
{
    SCENE_CoRBS,
    SCENE_Washington,
    OBJECT,
    SCENE_ICL,
    SCENE_TUM,
};
}

class CameraParam
{
    static constexpr int valid_range_beg_mm = 100;
    static constexpr int valid_range_end_mm = 3500;

    static constexpr float cor_flh = 468.60f/2.f;
    static constexpr float cor_flv = 468.61f/2.f;
    static constexpr float cor_cth = 318.27f/2.f;
    static constexpr float cor_ctv = 243.99f/2.f;

    static constexpr float was_flh = 570.3f/2.f;
    static constexpr float was_flv = 570.3f/2.f;
    static constexpr float was_cth = 320.f/2.f;
    static constexpr float was_ctv = 240.f/2.f;

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

    static constexpr float sample_range_clean = 3.0f;
    static constexpr float track_range_clean = 3.5f;
    static constexpr float sample_range_noisy = 1.5f;
    static constexpr float track_range_noisy = 1.5f;
    static constexpr float sample_range_corbs = 1.5f;
    static constexpr float track_range_corbs = 2.0f;

public:
    static int cameraType;
    static float flh()
    {
        if(cameraType==CameraType::SCENE_CoRBS)
            return cor_flh;
        else if(cameraType==CameraType::SCENE_Washington)
            return was_flh;
        else if(cameraType==CameraType::OBJECT)
            return obj_flh;
//        else if(cameraType==CameraType::ICL_NUIM_room1_noisy)
//            return icl_flh;
//        else if(cameraType==CameraType::TUM_freiburg3_long)
//            return tum_flh;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float flv()
    {
        if(cameraType==CameraType::SCENE_CoRBS)
            return cor_flv;
        else if(cameraType==CameraType::SCENE_Washington)
            return was_flv;
        else if(cameraType==CameraType::OBJECT)
            return obj_flv;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float cth()
    {
        if(cameraType==CameraType::SCENE_CoRBS)
            return cor_cth;
        else if(cameraType==CameraType::SCENE_Washington)
            return was_cth;
        else if(cameraType==CameraType::OBJECT)
            return obj_cth;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float ctv()
    {
        if(cameraType==CameraType::SCENE_CoRBS)
            return cor_ctv;
        else if(cameraType==CameraType::SCENE_Washington)
            return was_ctv;
        else if(cameraType==CameraType::OBJECT)
            return obj_ctv;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float sampleRange()
    {
        return 3.0f;
    }
    static float trackRange()
    {
        return 3.5f;
    }

    static int RangeBeg_mm() { return valid_range_beg_mm; }
    static int RangeEnd_mm() { return valid_range_end_mm; }
    static float RangeBeg_m() { return valid_range_beg_mm/1000.f; }
    static float RangeEnd_m() { return valid_range_end_mm/1000.f; }

};

#endif // CAMERA_PARAM_H
