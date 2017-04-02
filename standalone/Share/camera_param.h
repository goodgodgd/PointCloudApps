#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include "Share/exceptions.h"
#include "IO/FileReaders/rgbdposereader.h"

enum class CameraType
{
    SCENE_CoRBS,
    SCENE_Washington,
    OBJECT,
    SCENE_ICL,
    SCENE_TUM,
};

class CameraParam
{
    static constexpr int valid_range_beg_mm = 100;
    static constexpr int valid_range_end_mm = 3500;

    static constexpr float cor_flh = 468.60f;
    static constexpr float cor_flv = 468.61f;
    static constexpr float cor_cth = 318.27f;
    static constexpr float cor_ctv = 243.99f;

    static constexpr float was_flh = 570.3f;
    static constexpr float was_flv = 570.3f;
    static constexpr float was_cth = 320.f;
    static constexpr float was_ctv = 240.f;

    static constexpr float icl_flh = 481.2f;
    static constexpr float icl_flv = 480.f;
    static constexpr float icl_cth = 319.5f;
    static constexpr float icl_ctv = 239.5f;

    static constexpr float tum_flh = 525.f;
    static constexpr float tum_flv = 525.f;
    static constexpr float tum_cth = 319.5f;
    static constexpr float tum_ctv = 239.5f;

    static constexpr float obj_flh = 570.3f;
    static constexpr float obj_flv = 570.3f;
    static constexpr float obj_cth = 320.f;
    static constexpr float obj_ctv = 240.f;

    static constexpr float sample_range_clean = 3.0f;
    static constexpr float track_range_clean = 3.5f;
    static constexpr float sample_range_noisy = 1.5f;
    static constexpr float track_range_noisy = 1.5f;
    static constexpr float sample_range_corbs = 1.5f;
    static constexpr float track_range_corbs = 2.0f;

    static float cthor;
    static float ctver;
    static float flhor;
    static float flver;

public:
    static void SetCameraType(CameraType camtype)
    {
#ifdef SCALE_VAR
        const float scale = (float)SCALE_VAR;
#else
        const float scale = 2.f;
#endif

        if(camtype==CameraType::SCENE_CoRBS)
        {
            cthor = cor_cth/scale;
            ctver = cor_ctv/scale;
            flhor = cor_flh/scale;
            flver = cor_flv/scale;
        }
        else if(camtype==CameraType::SCENE_Washington)
        {
            cthor = was_cth/scale;
            ctver = was_ctv/scale;
            flhor = was_flh/scale;
            flver = was_flv/scale;
        }
        else if(camtype==CameraType::OBJECT)
        {
            cthor = obj_cth/scale;
            ctver = obj_ctv/scale;
            flhor = obj_flh;
            flver = obj_flv;
        }
        else
            throw TryFrameException("invalid camera ID");
    }

    static float flh() { return flhor; }
    static float flv() { return flver; }
    static float cth() { return cthor; }
    static float ctv() { return ctver; }
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
