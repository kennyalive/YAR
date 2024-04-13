#pragma once

#include "lib/bounding_box.h"
#include "lib/image.h"
#include "lib/vector.h"

struct Scene;
struct Scene_Context;

struct Renderer_Configuration {
    int thread_count = 0;
    std::string checkpoint_directory;
    bool rebuild_kdtree_cache = false;

    // Can be useful during debugging to vary random numbers and get configuration that
    // reproduces desired behavior.
    int rng_seed_offset = 0;

    bool pbrt_compatibility = false;
};

struct EXR_Attributes {
    std::string input_file;
    int spp = 0; // samples per pixel
    float variance = 0.f;

    // The attributes that might vary between render sessions.
    float load_time = 0.f;
    float render_time = 0.f;
};

struct EXR_Write_Params {
    bool enable_compression = false;
    bool enable_varying_attributes = false;
    bool dump_attributes = false;
    EXR_Attributes attributes;
};

void init_scene_context(const Scene& scene, const Renderer_Configuration& config, Scene_Context& scene_ctx);
Image render_scene(const Scene_Context& scene_ctx, double* variance_estimate, float* render_time);
bool write_openexr_image(const std::string& filename, const Image& image, const EXR_Write_Params& write_params);
