#include "std.h"
#include "lib/common.h"
#include "yar.h"

#include "glfw/glfw3.h"

struct Command_Line_Params
{
    bool requested_help_info = false;
    std::string data_dir;
    int gpu_index = -1;
    std::string input_file;
};

struct Window_State
{
    GLFWwindow* window = nullptr;
    YAR* yar = nullptr;
    int width = 960;
    int height= 720;
    bool active = false;
    bool vsync = false;
};

static Command_Line_Params parse_command_line(int argc, char** argv)
{
    Command_Line_Params params;
    for (int i = 1; i < argc; i++) {
        const bool last = (i == argc - 1);
        if (strcmp(argv[i], "-gpu") == 0) {
            if (last) {
                printf("-gpu parameter does not specify integer gpu index\n");
            }
            else {
                params.gpu_index = std::stoi(argv[i + 1]);
                i++;
            }
        }
        else if (strcmp(argv[i], "-data-dir") == 0) {
            if (last) {
                printf("-data-dir parameter does not specify location of program's data folder\n");
            }
            else {
                set_data_directory(argv[i+1]);
                i++;
            }
        }
        else if (strcmp(argv[i], "-help") == 0) {
            printf("%-15s Path to the data directory. Default: %s\n", "-data-dir", get_data_directory().string().c_str());
            printf("%-15s Help attempt (shows this information).\n", "-help");
            params.requested_help_info = true;
        }
        else {
            params.input_file = argv[i];
        }
    }
    return params;
}

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW error: %s\n", description);
}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_F11 || key == GLFW_KEY_ENTER && mods == GLFW_MOD_ALT) {
            static int last_window_xpos, last_window_ypos;
            static int last_window_width, last_window_height;

            VK_CHECK(vkDeviceWaitIdle(vk.device));
            GLFWmonitor* monitor = glfwGetWindowMonitor(window);
            if (monitor == nullptr) {
                Window_State* window_state = (Window_State*)glfwGetWindowUserPointer(window);
                glfwGetWindowPos(window, &last_window_xpos, &last_window_ypos);
                last_window_width = window_state->width;
                last_window_height = window_state->height;

                monitor = glfwGetPrimaryMonitor();
                const GLFWvidmode* mode = glfwGetVideoMode(monitor);
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            } else {
                glfwSetWindowMonitor(window, nullptr, last_window_xpos, last_window_ypos, last_window_width, last_window_height, 0);
            }
        } else if (key == GLFW_KEY_F10) {
            Window_State* window_state = (Window_State*)glfwGetWindowUserPointer(window);
            window_state->yar->toggle_ui();
        }
    }
}

static void check_if_swapchain_needs_something(Window_State& window_state)
{
    int width, height;
    glfwGetWindowSize(window_state.window, &width, &height);
    YAR& yar = *window_state.yar;

    bool recreate_swapchain = false;
    if (window_state.vsync != yar.vsync_enabled()) {
        window_state.vsync = yar.vsync_enabled();
        recreate_swapchain = true;
    } else if (window_state.width != width || window_state.height != height) {
        window_state.width = width;
        window_state.height = height;
        recreate_swapchain = true;
    }

    window_state.active = (width != 0 && height != 0);
    if (!window_state.active) {
        return; 
    }
    if (recreate_swapchain) {
        yar.recreate_swapchain();
    }
}

static int run_realtime_renderer(const Command_Line_Params& params)
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        error("glfwInit failed");
    }
    Window_State window_state;

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window_state.window = glfwCreateWindow(window_state.width, window_state.height, "YAR", nullptr, nullptr);
    ASSERT(window_state.window != nullptr);
    glfwSetKeyCallback(window_state.window, glfw_key_callback);

    YAR yar{};
    yar.initialize(window_state.window, params.gpu_index);

    window_state.yar = &yar;
    window_state.active = true;
    window_state.vsync = yar.vsync_enabled();
    glfwSetWindowUserPointer(window_state.window, &window_state);

    if (!params.input_file.empty()) {
        yar.load_project(params.input_file);
    }
    while (!glfwWindowShouldClose(window_state.window)) {
        if (window_state.active) {
            yar.run_frame();
        }
        glfwPollEvents();
        check_if_swapchain_needs_something(window_state);
    }
    yar.shutdown();
    glfwTerminate();
    return 0;
}

int main(int argc, char** argv)
{
    const Command_Line_Params command_line_params = parse_command_line(argc, argv);
    if (command_line_params.requested_help_info) {
        return 0;
    }
    run_realtime_renderer(command_line_params);
    return 0;
}
