#pragma once

#include "image_texture.h"
#include "light_sampling.h"
#include "kdtree.h"
#include "pixel_sampling.h"

#include "lib/light.h"
#include "lib/random.h"
#include "lib/utils.h"

class Camera;
class Image_Texture;
struct Scene;

// Array2D ids that can be supplied to pixel sampler's get_array2d() function.
struct Array2D_Ids {
    struct MIS_Info {
        int light_array2d_id = -1;
        int bsdf_array2d_id = -1;
    };
    std::vector<MIS_Info> rectangular_lights_sampling;
    std::vector<MIS_Info> sphere_lights_sampling;
};

struct Scene_Context {
    const Scene* scene = nullptr;
    const Camera* camera;
    const Scene_KdTree* acceleration_structure;
    Materials materials;
    std::vector<Image_Texture> textures;
    Stratified_Pixel_Sampler_Configuration pixel_sampler_config;
    Array2D_Ids array2d_ids;

    // Light data
    Lights lights;
    Environment_Light_Sampler environment_light_sampler;
    bool has_environment_light_sampler = false;
};

struct Thread_Context {
    Memory_Pool memory_pool;
    RNG rng;
    Stratified_Pixel_Sampler pixel_sampler;
    int current_pixel_sample_index = -1;
};
