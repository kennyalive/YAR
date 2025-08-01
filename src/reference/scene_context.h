#pragma once

#include "camera.h"
#include "kdtree.h"
#include "image_texture.h"
#include "light_sampling.h"
#include "pixel_sampling.h"

#include "lib/bounding_box.h"
#include "lib/light.h"
#include "lib/material.h"
#include "lib/raytracer_config.h"

struct Scene;

struct KdTree_Data {
    std::vector<Triangle_Mesh_Geometry_Data> triangle_mesh_geometry_data;
    std::vector<KdTree> geometry_kdtrees;
    Scene_Geometry_Data scene_geometry_data;
    KdTree scene_kdtree;

    void initialize(const Scene& scene, const std::vector<Image_Texture>& textures, bool rebuild_kdtree_cache);
};

struct MIS_Array_Info {
    int light_array_id = -1;
    int bsdf_wi_array_id = -1;
    int bsdf_scattering_array_id = -1;
    int array_size = 0;
};

struct Array2D_Registry {
    std::vector<MIS_Array_Info> rectangular_light_arrays;
    std::vector<MIS_Array_Info> sphere_light_arrays;
};

struct Scene_Context {
    std::string input_filename;
    std::string checkpoint_directory;
    int thread_count = 0;

    Bounds2i render_region;
    Raytracer_Config raytracer_config;
    Camera camera;

    KdTree_Data kdtree_data;

    // Materials
    Materials materials;
    std::vector<Image_Texture> textures;

    // Lights
    Lights lights;
    Environment_Light_Sampler environment_light_sampler;
    std::vector<Diffuse_Triangle_Mesh_Light_Sampler> triangle_mesh_light_samplers;

    // Samplers
    Stratified_Pixel_Sampler_Configuration pixel_sampler_config;
    Array2D_Registry array2d_registry; // registered 2d arrays of samples

    // Pbrt format support.
    bool pbrt3_scene = false;
    bool pbrt4_scene = false;
    bool pbrt_compatibility = false; // this flag has detailed documentation in main.cpp

    // Can be useful during debugging to vary random numbers and get configuration that
    // reproduces desired behavior.
    int rng_seed_offset = 0;
};
