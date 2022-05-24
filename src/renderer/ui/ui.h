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
    bool vsync = true;

    bool* raytracing = nullptr;
    bool* spp4 = nullptr;
    const GPU_Time_Scope* frame_time_scope = nullptr;

    Vector3 camera_position;
};

