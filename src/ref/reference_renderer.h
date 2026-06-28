#pragma once

#include "lib/image.h"
#include "lib/matrix.h"
#include "lib/raytracer_config.h"

struct Scene;
struct Scene_Context;

struct Reference_Renderer_Config
{
    int thread_count = 0;
    std::string checkpoint_directory;
    bool rebuild_kdtree_cache = false;

    // Can be useful during debugging to vary random numbers and get configuration that
    // reproduces desired behavior.
    int rng_seed_offset = 0;

    bool pbrt_compatibility = false;
};

struct Scene_Overrides
{
    std::optional<Raytracer_Config> raytracer_config;
    std::optional<Matrix3x4> camera_pose;
};

struct EXR_Attributes
{
    std::string input_file;
    int spp = 0; // samples per pixel
    float variance = 0.f;

    // The attributes that might vary between render sessions.
    float load_time = 0.f;
    float render_time = 0.f;
};

struct EXR_Write_Params
{
    bool enable_compression = false;
    bool enable_varying_attributes = false;
    bool dump_attributes = false;
    EXR_Attributes attributes;
};

void init_scene_context(
    Scene_Context& scene_ctx,
    const Scene& scene,
    const Reference_Renderer_Config& config,
    const Scene_Overrides& overrides = {}
);

Image render_scene(const Scene_Context& scene_ctx, double* variance_estimate, float* render_time);
bool write_openexr_image(const std::string& filename, const Image& image, const EXR_Write_Params& write_params);
