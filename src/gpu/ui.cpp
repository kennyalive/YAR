#include "ui.h"

#include "gpu/vk.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_vulkan.h"
#include "imgui/imgui_impl_glfw.h"

static const ImVec4 accent_fill(0.10f, 0.35f, 0.32f, 1.0f);

void UI::apply_style() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FontSizeBase = 18.0f;
    style.WindowPadding = ImVec2(14.0f, 12.0f);
    style.FramePadding = ImVec2(10.0f, 6.0f);
    style.ItemSpacing = ImVec2(10.0f, 7.0f);
    style.ItemInnerSpacing = ImVec2(7.0f, 4.0f);
    style.WindowRounding = 8.0f;
    style.PopupRounding = 6.0f;
    style.FrameRounding = 5.0f;
    style.GrabRounding = 4.0f;
    style.FrameBorderSize = 1.0f;

    const ImVec4 panel(0.09f, 0.10f, 0.11f, 0.97f);
    const ImVec4 control(0.18f, 0.21f, 0.22f, 1.0f);
    const ImVec4 hovered(0.14f, 0.42f, 0.38f, 1.0f);
    const ImVec4 pressed(0.09f, 0.29f, 0.27f, 1.0f);
    const ImVec4 accent(0.40f, 0.86f, 0.75f, 1.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = panel;
    colors[ImGuiCol_PopupBg] = panel;
    colors[ImGuiCol_TitleBg] = panel;
    colors[ImGuiCol_TitleBgActive] = control;
    colors[ImGuiCol_Border] = ImVec4(0.33f, 0.38f, 0.39f, 1.0f);
    colors[ImGuiCol_Separator] = colors[ImGuiCol_Border];
    colors[ImGuiCol_FrameBg] = control;
    colors[ImGuiCol_FrameBgHovered] = hovered;
    colors[ImGuiCol_FrameBgActive] = pressed;
    colors[ImGuiCol_CheckboxSelectedBg] = accent_fill;
    colors[ImGuiCol_CheckMark] = accent;
    colors[ImGuiCol_SliderGrab] = accent;
    colors[ImGuiCol_SliderGrabActive] = accent;
    colors[ImGuiCol_Button] = control;
    colors[ImGuiCol_ButtonHovered] = hovered;
    colors[ImGuiCol_ButtonActive] = pressed;
    colors[ImGuiCol_Header] = control;
    colors[ImGuiCol_HeaderHovered] = hovered;
    colors[ImGuiCol_HeaderActive] = pressed;
    colors[ImGuiCol_ResizeGrip] = control;
    colors[ImGuiCol_ResizeGripHovered] = hovered;
    colors[ImGuiCol_ResizeGripActive] = accent;
    colors[ImGuiCol_TextSelectedBg] = accent_fill;
    colors[ImGuiCol_NavCursor] = accent;
}

static void show_time_scope(const Vk_Timer* timer) {
    ImGui::Text("%s time : %.2f ms", timer->name, timer->duration_ms);
    for (const Vk_Timer* nested_timer : timer->nested_timers) {
        show_time_scope(nested_timer);
    }
}

UI_Actions UI::run_imgui() {
    UI_Actions actions;
    renderer_changed = false;

    ImGuiIO& io = ImGui::GetIO();

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (show_ui) {
        const float DISTANCE = 10.0f;
        static int corner = 0;

        ImVec2 window_pos = ImVec2((corner & 1) ? ImGui::GetIO().DisplaySize.x - DISTANCE : DISTANCE,
                                   (corner & 2) ? ImGui::GetIO().DisplaySize.y - DISTANCE : DISTANCE);

        ImVec2 window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);

        if (corner != -1) {
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        }

        if (ImGui::Begin("UI", &show_ui, 
            (corner != -1 ? ImGuiWindowFlags_NoMove : 0) | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav)) {
            ImGui::SeparatorText("Renderer");
            renderer_changed |= ImGui::RadioButton("Direct lighting", &rendering_algorithm, 0);
            ImGui::SameLine();
            renderer_changed |= ImGui::RadioButton("Path tracing", &rendering_algorithm, 1);
            ImGui::Checkbox("Vertical sync", &vsync);

            ImGui::SeparatorText("Reference render");
            ImGui::PushItemWidth(6.0f * ImGui::GetFontSize());
            ImGui::InputScalar("Threads", ImGuiDataType_U32, &ref_params.thread_count);
            ImGui::InputScalar("Samples per pixel", ImGuiDataType_U32, &ref_params.spp);
            ImGui::PopItemWidth();
            ImGui::BeginDisabled(reference_renderer_running || !scene_loaded);
            ImGui::PushStyleColor(ImGuiCol_Button, accent_fill);
            if (ImGui::Button("Render reference image", ImVec2(-FLT_MIN, 0.0f))) {
                actions.reference_render_requested = true;
            }
            ImGui::PopStyleColor();
            ImGui::EndDisabled();

            ImGui::SeparatorText("Project file");
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputText("##Project file", project_file, sizeof(project_file));
            if (ImGui::Button("Load")) {
                actions.load_project = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Unload")) {
                actions.unload_project = true;
            }

            ImGui::Separator();
            ImGui::Text("%.1f FPS (%.3f ms/frame)", io.Framerate, 1000.0f / io.Framerate);
            if (ImGui::CollapsingHeader("Diagnostics")) {
                show_time_scope(frame_time_scope);
                ImGui::Text("Camera position: %.2f %.2f %.2f", camera_position.x, camera_position.y, camera_position.z);
            }

            if (ImGui::BeginPopupContextWindow()) {
                if (ImGui::MenuItem("Custom",       NULL, corner == -1)) corner = -1;
                if (ImGui::MenuItem("Top-left",     NULL, corner == 0)) corner = 0;
                if (ImGui::MenuItem("Top-right",    NULL, corner == 1)) corner = 1;
                if (ImGui::MenuItem("Bottom-left",  NULL, corner == 2)) corner = 2;
                if (ImGui::MenuItem("Bottom-right", NULL, corner == 3)) corner = 3;
                if (ImGui::MenuItem("Close")) show_ui = false;
                ImGui::EndPopup();
            }
        }
        ImGui::End();
    }
    return actions;
}
