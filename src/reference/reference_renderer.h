#pragma once

#include "lib/bounding_box.h"

struct Renderer_Options {
    int thread_count = 0;

    Bounds2i render_region;
    bool crop_image_by_render_region = false;
    int render_tile_index = -1;

    // Can be useful during debugging to vary random numbers and get configuration that
    // reproduces desired behavior.
    int rng_seed_offset = 0;

    // Can be used to match output of the renderer that uses left-handed coordinate system.
    bool flip_image_horizontally = false;

    bool force_rebuild_kdtree_cache = false;

    std::string output_filename_suffix;
};

void cpu_renderer_render(const std::string& input_file, const Renderer_Options& options);
