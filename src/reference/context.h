#pragma once

#include "image_texture.h"
#include "light_sampling.h"
#include "kdtree.h"
#include "pixel_sampling.h"
#include "shading_context.h"

#include "lib/light.h"
#include "lib/random.h"
#include "lib/utils.h"

class Camera;
class Image_Texture;
struct Scene;

struct MIS_Array_Info {
    int light_array_id = -1;
    int bsdf_array_id = -1;
    int array_size = 0;
};

struct Array2D_Registry {
    std::vector<MIS_Array_Info> rectangular_light_arrays;
    std::vector<MIS_Array_Info> sphere_light_arrays;
};

struct Scene_Context {
    const Scene* scene = nullptr;
    const Camera* camera;
    const Scene_KdTree* acceleration_structure;
    Materials materials;
    std::vector<Image_Texture> textures;
    Stratified_Pixel_Sampler_Configuration pixel_sampler_config;
    Array2D_Registry array2d_registry; // registered 2d arrays of samples

    // Light data
    Lights lights;
    Environment_Light_Sampler environment_light_sampler;
    bool has_environment_light_sampler = false;
};

struct Thread_Context {
    Memory_Pool memory_pool;
    RNG rng;
    Stratified_Pixel_Sampler pixel_sampler;
    Shading_Context shading_context;

    // Variance estimation.
    double variance_accumulator = 0.0;
    int64_t variance_count = 0;
};
