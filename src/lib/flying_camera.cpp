#include "std.h"
#include "common.h"
#include "flying_camera.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"

constexpr float Yaw_Rotate_Speed = radians(90.f);
constexpr float Pitch_Rotate_Speed = radians(90.f);

void Flying_Camera::initialize(Matrix3x4 camera_pose, bool z_is_up) {
    this->camera_pose = camera_pose;
    this->z_is_up = z_is_up;
    camera_transform_changes_handedness = is_transform_changes_handedness(camera_pose);
}

void Flying_Camera::update(double dt) {
    int forward_motion = 0;
    int right_motion = 0;
    int up_motion = 0;

    if (!ImGui::GetIO().WantCaptureKeyboard) {
        if (ImGui::IsKeyDown(GLFW_KEY_D)) {
            right_motion += 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_A)) {
            right_motion -= 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_W) || ImGui::IsKeyDown(GLFW_KEY_UP)) {
            forward_motion += 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_S) || ImGui::IsKeyDown(GLFW_KEY_DOWN)) {
            forward_motion -= 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_E)) {
            up_motion += 1;
        }
        if (ImGui::IsKeyDown(GLFW_KEY_Q)) {
            up_motion -= 1;
        }
    }

    float yaw_delta = 0.f;
    float pitch_delta = 0.f;

    if (!ImGui::GetIO().WantCaptureMouse) {
        if (last_mouse_pos.x == -1.f) { // initialize
            last_mouse_pos = {ImGui::GetMousePos().x, ImGui::GetMousePos().y};
        }
        Vector2 mouse_pos = {ImGui::GetMousePos().x, ImGui::GetMousePos().y};
        Vector2 mouse_delta = mouse_pos - last_mouse_pos;
        last_mouse_pos = mouse_pos;

        if (mouse_delta != Vector2_Zero && ImGui::IsMouseDown(0) && !ImGui::IsMouseClicked(0)) {
            yaw_delta = (-mouse_delta.x / ImGui::GetWindowWidth()) * Yaw_Rotate_Speed;
            pitch_delta = (-mouse_delta.y / ImGui::GetWindowHeight()) * Pitch_Rotate_Speed;
        }
        if (ImGui::IsMouseDown(0)) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_None);
        } else {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
        }
        if (ImGui::GetIO().MouseWheel != 0.f) {
            if (ImGui::GetIO().MouseWheel > 0)
                speed_multiplier *= 1.5f;
            else
                speed_multiplier /= 1.5f;
        }
    }

    if (forward_motion || right_motion || up_motion) {
        Vector3 position = camera_pose.get_column(3);
        float speed = speed_multiplier * (ImGui::IsKeyDown(GLFW_KEY_LEFT_SHIFT) ? 3.0f : 1.f);
        float distance_delta = float(speed * dt);

        position += camera_pose.get_column(0) * (distance_delta * right_motion);
        if (z_is_up) {
            position += camera_pose.get_column(1) * float(distance_delta * forward_motion);
            position += Vector3(0, 0, 1) * float(distance_delta * up_motion);
        }
        else { // y_is_up
            position += -camera_pose.get_column(2) * float(distance_delta * forward_motion);
            position += Vector3(0, 1, 0) * float(distance_delta * up_motion);
        }
        camera_pose.set_column(3, position);
    }
    if (yaw_delta || pitch_delta) {
        Vector3 temp_position = camera_pose.get_column(3);
        camera_pose.set_column(3, Vector3_Zero);

        Matrix3x4 pitch_rotation = rotate_x(Matrix3x4::identity, pitch_delta);

        Matrix3x4 yaw_rotation;
        if (z_is_up) {
            yaw_rotation = rotate_z(Matrix3x4::identity, yaw_delta * (camera_transform_changes_handedness ? -1.f : 1.f));
        }
        else { // y_is_up
            yaw_rotation = rotate_y(Matrix3x4::identity, yaw_delta * (camera_transform_changes_handedness ? -1.f : 1.f));
        }

        camera_pose = yaw_rotation * camera_pose * pitch_rotation;
        camera_pose.set_column(3, temp_position);
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
