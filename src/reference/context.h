#pragma once

#include "image_texture.h"
#include "kdtree.h"
#include "sampling.h"

#include "lib/geometry.h"
#include "lib/light.h"
#include "lib/utils.h"

class Camera;

struct Render_Context { // Scene_Context rename ???
    Bounds2i sample_bounds;
    const Camera* camera;
    const Scene_KdTree* acceleration_structure;
    Lights lights;
    Materials materials;
    std::vector<Image_Texture> textures;
    std::vector<Distribution_2D_Sampling> environment_lights_sampling;

};

struct Thread_Context {
    Memory_Pool memory_pool;
};
