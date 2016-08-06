#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include "Share/exceptions.h"

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

    static constexpr float icl_flh = 525.f/2.f;
    static constexpr float icl_flv = 525.f/2.f;
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

public:
    static int camType;
    static float flh()
    {
        if(camType==CameraType::SCENE_ICL)
            return icl_flh;
        else if(camType==CameraType::SCENE_TUM)
            return icl_flh;
        else if(camType==CameraType::OBJECT)
            return obj_flh;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float flv()
    {
        if(camType==CameraType::SCENE_ICL)
            return icl_flv;
        else if(camType==CameraType::SCENE_TUM)
            return icl_flv;
        else if(camType==CameraType::OBJECT)
            return obj_flv;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float cth()
    {
        if(camType==CameraType::SCENE_ICL)
            return icl_cth;
        else if(camType==CameraType::SCENE_TUM)
            return icl_cth;
        else if(camType==CameraType::OBJECT)
            return obj_cth;
        else
            throw TryFrameException("invalid camera ID");
    }
    static float ctv()
    {
        if(camType==CameraType::SCENE_ICL)
            return icl_ctv;
        else if(camType==CameraType::SCENE_TUM)
            return icl_ctv;
        else if(camType==CameraType::OBJECT)
            return obj_ctv;
        else
            throw TryFrameException("invalid camera ID");
    }

    static int RangeBeg_mm() { return valid_range_beg_mm; }
    static int RangeEnd_mm() { return valid_range_end_mm; }
    static float RangeBeg_m() { return valid_range_beg_mm/1000.f; }
    static float RangeEnd_m() { return valid_range_end_mm/1000.f; }

};

#endif // CAMERA_PARAM_H
