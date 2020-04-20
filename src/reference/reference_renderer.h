#pragma once

struct Renderer_Options {
    int thread_count = 0;
};

void render_reference_image(const std::string& input_file, const Renderer_Options& options);
