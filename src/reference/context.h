#pragma once

#include "camera.h"
#include "image_texture.h"
#include "light_sampling.h"
#include "kdtree.h"
#include "pixel_sampling.h"
#include "shading_context.h"

#include "lib/light.h"
#include "lib/material.h"
#include "lib/random.h"
#include "lib/raytracer_config.h"
#include "lib/utils.h"

class Camera;
class Image_Texture;
struct Scene;
struct Renderer_Options;

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
    std::string input_filename;

    Raytracer_Config raytracer_config;
    Camera camera;
    const KdTree* acceleration_structure = nullptr;

    // Materials
    Materials materials;
    std::vector<Image_Texture> textures;

    // Lights
    Lights lights;
    Environment_Light_Sampler environment_light_sampler;
    bool has_environment_light_sampler = false;

    // Samplers
    Stratified_Pixel_Sampler_Configuration pixel_sampler_config;
    Array2D_Registry array2d_registry; // registered 2d arrays of samples
};

struct Path_Context {
    int bounce_count = 0; // current number of bounces
    int perfect_specular_bounce_count = 0;
};

struct Thread_Context {
    Memory_Pool memory_pool;
    RNG rng;
    Stratified_Pixel_Sampler pixel_sampler;

    const Renderer_Options* renderer_options = nullptr;
    const Scene_Context* scene_context = nullptr;
    Path_Context path_context;
    Shading_Context shading_context;

    // TODO: until we implement proper handling of nested dielectrics we make assumption
    // that we don't have nested dielectrics and after we start tracing inside dielectric
    // the only possible hit can be with the same dielectric material for exit event. Here
    // we track current dielectric material to assert this convention and also to determine
    // if it's enter or exit event.
    Material_Handle current_dielectric_material;
};
