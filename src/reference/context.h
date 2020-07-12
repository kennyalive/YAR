#pragma once

#include "image_texture.h"
#include "light_sampling.h"
#include "kdtree.h"

#include "lib/light.h"
#include "lib/utils.h"

class Camera;
class Image_Texture;

struct Scene_Context {
    const Camera* camera;
    const Scene_KdTree* acceleration_structure;
    Lights lights;
    Materials materials;
    std::vector<Image_Texture> textures;

    Environment_Light_Sampler environment_light_sampler;
    bool has_environment_light_sampler = false;
};

struct Thread_Context {
    Memory_Pool memory_pool;
};
