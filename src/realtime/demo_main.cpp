#include "demo.h"

#include "sdl/SDL.h"
#include "sdl/SDL_syswm.h"

#include "imgui/imgui.h"
#include "imgui/impl/imgui_impl_sdl.h"

static bool toogle_fullscreen   = false;
static bool handle_resize       = false;

static bool process_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);

        if (event.type == SDL_QUIT)
            return false;

        if (event.type == SDL_KEYDOWN) {
            SDL_Scancode scancode = event.key.keysym.scancode;

            if (scancode == SDL_SCANCODE_ESCAPE)
                return false;

            if (scancode == SDL_SCANCODE_F11 || scancode == SDL_SCANCODE_RETURN && (SDL_GetModState() & KMOD_LALT) != 0)
                toogle_fullscreen = true;
        }

        if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED)
            handle_resize = true;
    }
    return true;
}

int run_vk_demo(bool enable_validation_layers, bool use_debug_names) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
        error("SDL_Init error");

    struct On_Exit {~On_Exit() { SDL_Quit(); }} exit_action;

    Vk_Create_Info vk_create_info{};
    vk_create_info.enable_validation_layers = enable_validation_layers;
    vk_create_info.use_debug_names = use_debug_names;

    // Create window.
    SDL_Window* the_window = SDL_CreateWindow("Vulkan demo",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 720, 720,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (the_window == nullptr)
        error("failed to create SDL window");

    SDL_VERSION(&vk_create_info.windowing_system_info.version);
    if (SDL_GetWindowWMInfo(the_window, &vk_create_info.windowing_system_info) == SDL_FALSE)
        error("failed to get platform specific window information");

    // Initialize demo.
    Vk_Demo demo{};
    demo.initialize(vk_create_info, the_window);

    bool prev_vsync = demo.vsync_enabled();
    bool handle_vsync_toggle = false;

    // Run main loop.
    while (process_events()) {
        if (toogle_fullscreen) {
            if (SDL_GetWindowFlags(the_window) & SDL_WINDOW_FULLSCREEN_DESKTOP)
                SDL_SetWindowFullscreen(the_window, 0);
            else
                SDL_SetWindowFullscreen(the_window, SDL_WINDOW_FULLSCREEN_DESKTOP);

            process_events();
            toogle_fullscreen = false;
        }

        if (handle_resize || handle_vsync_toggle) {
            if (vk.swapchain_info.handle != VK_NULL_HANDLE) {
                VK_CHECK(vkDeviceWaitIdle(vk.device));
                demo.release_resolution_dependent_resources();
                vk_release_resolution_dependent_resources();
            }
            handle_resize = false;
            handle_vsync_toggle = false;
        }

        if ((SDL_GetWindowFlags(the_window) & SDL_WINDOW_MINIMIZED) == 0) {
            if (vk.swapchain_info.handle == VK_NULL_HANDLE) {
                vk_restore_resolution_dependent_resources(demo.vsync_enabled());
                demo.restore_resolution_dependent_resources();
            }

            demo.run_frame();

            if (prev_vsync != demo.vsync_enabled()) {
                prev_vsync = demo.vsync_enabled();
                handle_vsync_toggle = true;
            }
        }
        SDL_Delay(1);
    }

    demo.shutdown();
    return 0;
}
