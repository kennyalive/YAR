#pragma once

#include "kernels/apply_tone_mapping.h"
#include "kernels/copy_to_swapchain.h"
#include "kernels/patch_materials.h"
#include "kernels/direct_lighting.h"
#include "kernels/path_tracing.h"

#include "descriptor_heap.h"
#include "descriptors.h"
#include "geometry.h"
#include "vk.h"

#include "ui.h"

#include "lib/flying_camera.h"
#include "lib/matrix.h"
#include "lib/scene.h"

struct GLFWwindow;
struct Reference_Renderer_Config;
struct Scene_Overrides;

class Renderer {
public:
    void initialize(GLFWwindow* glfw_window, int gpu_index);
    void shutdown();
    void recreate_swapchain();

    bool vsync_enabled() const { return ui.vsync; }
    void toggle_ui() { ui.show_ui = !ui.show_ui; }

    void load_project(const std::string& input_file);
    void run_frame();

private:
    void release_resolution_dependent_resources();
    void restore_resolution_dependent_resources();
    void create_default_textures();

    void draw_frame();
    void draw_raytraced_image();
    void tone_mapping();
    void draw_imgui();
    void copy_output_image_to_swapchain();

    void start_reference_renderer();
    void do_run_reference_renderer(const Reference_Renderer_Config& reference_renderer_config, const Scene_Overrides& overrides);
    void wait_for_reference_renderer();

private:
    uint32_t frame_index = 0;
    uint32_t accumulation_index = 0;
    bool spp4 = false;

    Flying_Camera flying_camera;

    VkPhysicalDeviceDescriptorHeapPropertiesEXT descriptor_heap_properties;

    Vk_Image output_image;
    Vk_Image tonemapped_image;

    std::vector<GPU_Mesh> gpu_meshes;

    Descriptor_Heap descriptor_heap;
    Descriptors descriptors;

    struct GPU_Scene_Resources {
        std::vector<Vk_Image> images_2d;
        Vk_Buffer instance_info_buffer;
        Vk_Buffer scene_info_buffer;
        Vk_Buffer point_lights;
        Vk_Buffer directional_lights;
        Vk_Buffer rect_lights;
        Vk_Buffer lambertian_material_buffer;
    } gpu_scene;

    Apply_Tone_Mapping apply_tone_mapping;
    Copy_To_Swapchain copy_to_swapchain;
    Patch_Materials patch_materials;
    Direct_Lighting direct_lighting;
    Path_Tracing path_tracing;

    Vk_Time_Keeper time_keeper;
    struct {
        Vk_Timer* frame;
        Vk_Timer* draw;
        Vk_Timer* tone_map;
        Vk_Timer* ui;
        Vk_Timer* compute_copy;
    } gpu_timers;

    bool project_loaded = false;
    Scene scene;
    UI ui;

    std::atomic_bool reference_renderer_running = false;
    std::jthread reference_renderer_thread;
};
