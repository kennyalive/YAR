#pragma once

#include "lib/vector.h"

struct GPU_Time_Scope;

struct UI {
    void run_imgui();

    struct UI_Result {
        bool reference_render_requested;
    };
    UI_Result ui_result;
    bool show_ui = true;
    bool vsync = false;

    bool* spp4 = nullptr;
    const GPU_Time_Scope* frame_time_scope = nullptr;

    Vector3 camera_position;

    struct Reference_Renderer_Params {
        uint32_t thread_count = 0;
        uint32_t spp = 4;
    };
    Reference_Renderer_Params ref_params;
};

