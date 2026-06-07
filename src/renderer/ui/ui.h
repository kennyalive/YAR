#pragma once

#include "lib/vector.h"
#include "lib/raytracer_config.h"

struct Vk_Timer;

struct UI_Actions {
    bool reference_render_requested = false;
};

struct UI {
    UI_Actions run_imgui();

    bool reset_accumulation = false;

    bool show_ui = true;
    bool vsync = true;
    int rendering_algorithm = 1;

    // Externally provide state displayed or used by the UI.
    bool reference_renderer_running = false;
    bool* spp4 = nullptr;
    const Vk_Timer* frame_time_scope = nullptr;
    Vector3 camera_position;

    struct Reference_Renderer_Params {
        uint32_t thread_count = 0;
        uint32_t spp = 4;
    };
    Reference_Renderer_Params ref_params;
};
