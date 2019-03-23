#pragma once

#include "lib/geometry.h"
#include "lib/matrix.h"
#include "lib/vector.h"

struct Scene_Data;

struct Reference_Renderer_Input {
    Vector2i image_resolution;
    Bounds2i render_region;
    Matrix3x4 camera_to_world;
    bool parallel_rendering;
    const Scene_Data* scene_data;
};

void render_reference_image(const Reference_Renderer_Input& input);
