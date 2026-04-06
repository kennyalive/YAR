#pragma once

#include "lib/vector.h"
#include "lib/raytracer_config.h"

struct Vk_Timer;

struct UI {
    void run_imgui();

    struct UI_Result {
        bool reference_render_requested;
    };
    UI_Result ui_result;
    bool show_ui = true;
    bool vsync = true;
    int rendering_algorithm = 1;

    bool* spp4 = nullptr;
    const Vk_Timer* frame_time_scope = nullptr;

    Vector3 camera_position;

    struct Reference_Renderer_Params {
        uint32_t thread_count = 0;
        uint32_t spp = 4;
    };
    Reference_Renderer_Params ref_params;
};
