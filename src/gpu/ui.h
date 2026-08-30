#pragma once

#include "lib/vector.h"
#include "lib/raytracer_config.h"

struct Vk_Timer;

struct UI_Actions
{
    bool reference_render_requested = false;
    bool load_project = false;
    bool unload_project = false;
};

struct UI
{
    UI_Actions run_imgui();

    bool show_ui = true;
    bool vsync = true;

    bool renderer_changed = false;
    int rendering_algorithm = 1;

    // Externally provide state displayed or used by the UI.
    bool scene_loaded = false;
    char project_file[512] = {}; // ImGui edits it in place; load_project rejects paths that do not fit
    bool reference_renderer_running = false;
    const Vk_Timer* frame_time_scope = nullptr;
    Vector3 camera_position;

    struct Reference_Renderer_Params {
        uint32_t thread_count = 0;
        uint32_t spp = 4;
    };
    Reference_Renderer_Params ref_params;
};
