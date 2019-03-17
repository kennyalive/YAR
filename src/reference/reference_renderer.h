#pragma once

#include "lib/geometry.h"
#include "lib/matrix.h"

struct Scene_Data;

struct Render_Reference_Image_Params {
    Vector2i image_resolution;
    Bounds2i render_region;
    const Scene_Data* scene_data;
    Matrix3x4 camera_to_world_vk;
    bool parallel_render = false;
};

void render_reference_image(const Render_Reference_Image_Params& params, bool* active);
