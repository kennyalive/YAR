#pragma once

#include "image_texture.h"
#include "kdtree.h"

#include "lib/geometry.h"
#include "lib/light.h"
#include "lib/utils.h"

class Camera;

struct Render_Context {
    Bounds2i sample_bounds;
    const Camera* camera;
    const Scene_KdTree* acceleration_structure;
    Lights lights;
    Materials materials;
    std::vector<Image_Texture> textures;
};

struct Thread_Context {
    Memory_Pool memory_pool;
};
