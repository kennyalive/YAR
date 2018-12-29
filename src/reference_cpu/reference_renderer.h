#pragma once

#include "lib/matrix.h"

struct Scene_Data;

struct Render_Reference_Image_Params {
    int w, h;
    const Scene_Data* scene_data;
    Matrix3x4 camera_to_world_vk;
};

void render_reference_image(const Render_Reference_Image_Params& params, bool* active);
