#include "std.h"
#include "common.h"
#include "flying_camera.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"

constexpr float X_Speed = 1.0f;
constexpr float Y_Speed = 1.0f;
constexpr float Z_Speed = 1.0f;

constexpr float XAxis_Rotate_Speed = radians(90.f);
constexpr float ZAxis_Rotate_Speed = radians(90.f);

void Flying_Camera::initialize(Matrix3x4 camera_pose) {
    this->camera_pose = camera_pose;
    velocity = {X_Speed, Y_Speed, Z_Speed};
}

void Flying_Camera::update(double dt) {
    int x_motion_sign = 0;
    int y_motion_sign = 0;
    int z_motion_sign = 0;

    if (!ImGui::GetIO().WantCaptureKeyboard) {
        if (ImGui::IsKeyDown(GLFW_KEY_D)) {
            x_motion_sign += 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_A)) {
            x_motion_sign -= 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_W) || ImGui::IsKeyDown(GLFW_KEY_UP)) {
            y_motion_sign += 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_S) || ImGui::IsKeyDown(GLFW_KEY_DOWN)) {
            y_motion_sign -= 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_E)) {
            z_motion_sign += 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_Q)) {
            z_motion_sign -= 1;
        }
    }

    float dx_angle = 0.f;
    float dz_angle = 0.f;

    if (!ImGui::GetIO().WantCaptureMouse) {
        if (last_mouse_pos.x == -1.f) { // initialize
            last_mouse_pos = {ImGui::GetMousePos().x, ImGui::GetMousePos().y};
        }
        Vector2 mouse_pos = {ImGui::GetMousePos().x, ImGui::GetMousePos().y};
        Vector2 mouse_delta = mouse_pos - last_mouse_pos;
        last_mouse_pos = mouse_pos;

        if (mouse_delta != Vector2_Zero && ImGui::IsMouseDown(0)) {
            dz_angle = (-mouse_delta.x / ImGui::GetWindowWidth()) * ZAxis_Rotate_Speed;
            dx_angle = (-mouse_delta.y / ImGui::GetWindowHeight()) * XAxis_Rotate_Speed;
        }
        if (ImGui::IsMouseDown(0)) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_None);
        } else {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
        }
        if (ImGui::GetIO().MouseWheel != 0.f) {
            if (ImGui::GetIO().MouseWheel > 0)
                velocity *= 1.5f;
            else
                velocity /= 1.5f;
        }
    }

    if (x_motion_sign || y_motion_sign || z_motion_sign) {
        Vector3 position = camera_pose.get_column(3);
        float velocity_scale = ImGui::IsKeyDown(GLFW_KEY_LEFT_SHIFT) ? 3.0f : 1.f;
        Vector3 movement = velocity * float(velocity_scale * dt);
        position += camera_pose.get_column(0) * float(movement.x * x_motion_sign);
        position += camera_pose.get_column(1) * float(movement.y * y_motion_sign);
        position += Vector3(0, 0, 1) * float(movement.z * z_motion_sign);
        camera_pose.set_column(3, position);
    }
    if (dx_angle || dz_angle) {
        Vector3 position = camera_pose.get_column(3);
        camera_pose.set_column(3, Vector3_Zero);
        Matrix3x4 z_rotation = rotate_z(Matrix3x4::identity, dz_angle);
        Matrix3x4 x_rotation = rotate_x(Matrix3x4::identity, dx_angle);
        camera_pose = z_rotation * camera_pose * x_rotation;
        camera_pose.set_column(3, position);
    }
}

Matrix3x4 Flying_Camera::get_view_transform() const {
    Vector3 x_axis = camera_pose.get_column(0);
    Vector3 y_axis = camera_pose.get_column(1);
    Vector3 z_axis = camera_pose.get_column(2);
    Vector3 position = camera_pose.get_column(3);

    Matrix3x4 m;
    m.set_row(0, Vector4(x_axis, -dot(position, x_axis)));
    m.set_row(1, Vector4(y_axis, -dot(position, y_axis)));
    m.set_row(2, Vector4(z_axis, -dot(position, z_axis)));
    return m;
}

Matrix3x4 Flying_Camera::get_camera_pose() const {
    return camera_pose;
}
