#pragma once

struct YAR_Project;

struct Renderer_Options {
    int cpu_core_count = 0;
};

void render_reference_image(const YAR_Project& project, const Renderer_Options& options);
