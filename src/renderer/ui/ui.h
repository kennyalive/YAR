#pragma once

// TODO: temporary here
struct GPU_Time_Scope;
struct GPU_Times {
    GPU_Time_Scope* frame;
    GPU_Time_Scope* draw;
    GPU_Time_Scope* ui;
    GPU_Time_Scope* compute_copy;
};

struct UI {
    void run_imgui();

    struct UI_Result {
        bool raytracing_toggled;
        bool reference_render_requested;
    };
    UI_Result ui_result;
    bool show_ui = true;
    bool vsync = true;

    bool* raytracing = nullptr;
    bool* spp4 = nullptr;
    const GPU_Times* gpu_times;
};
