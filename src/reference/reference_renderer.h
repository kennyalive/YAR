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
};

struct EXR_Custom_Attributes {
    std::string input_file;
    int spp = 0; // samples per pixel
    float variance = 0.f;

    // Here are the attributes that vary between render sessions. We store such
    // attributes in the output file only if --openexr-varying-attributes command
    // line option is specified. The reason why we do not always write them is to
    // keep output deterministic by default.
    float load_time = 0.f;
    float render_time = 0.f;
};

struct EXR_Write_Params {
    // Enables OpenEXR feature to store image data in compressed form (zip)
    bool enable_compression = false;

    // This option disables generation of openexr custom attributes that vary between render sessions.
    // Examples of varying attributes: render time, variance.
    // Examples of non-varying attributes: output file name, per pixel sample count.
    bool disable_varying_attributes = false;

    EXR_Custom_Attributes custom_attributes;
};

void init_scene_context(const Scene& scene, const Renderer_Configuration& config, Scene_Context& scene_ctx);
Image render_scene(const Scene_Context& scene_ctx, double* variance_estimate, float* render_time);
bool write_openexr_image(const std::string& filename, const Image& image, const EXR_Write_Params& write_params);
