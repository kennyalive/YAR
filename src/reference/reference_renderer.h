#pragma once

#include "lib/bounding_box.h"
#include "lib/vector.h"

struct Renderer_Options {
    int thread_count = 0;

    Bounds2i render_region;
    bool crop_image_by_render_region = false;

    // Can be useful during debugging to vary random numbers and get configuration that
    // reproduces desired behavior.
    int rng_seed_offset = 0;

    // Can be used to match output of the renderer that uses left-handed coordinate system.
    bool flip_image_horizontally = false;

    bool force_rebuild_kdtree_cache = false;

    // This option disables generation of openexr custom attributes that vary between render sessions.
    // Examples of varying attributes: render time, variance.
    // Examples of non-varying attributes: output file name, per pixel sample count.
    bool openexr_disable_varying_attributes = false;

    // Enables OpenEXR feature to store image data in compressed form (zip)
    bool openexr_enable_compression = false;

    std::string output_directory;
    std::string output_filename_suffix;

    int samples_per_pixel = 0; // overrides project settings
    Vector2i film_resolution; // overrides project settings
};

void cpu_renderer_render(const std::string& input_file, const Renderer_Options& options);
