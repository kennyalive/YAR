#pragma once

#include "lib/bounding_box.h"

struct Renderer_Options {
    int thread_count = 0;
    Bounds2i render_region;
    int render_tile_index = -1;
};

void render_reference_image(const std::string& input_file, const Renderer_Options& options);
