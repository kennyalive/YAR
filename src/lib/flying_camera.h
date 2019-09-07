#pragma once

#include "matrix.h"

class Flying_Camera {
public:
    void initialize(Matrix3x4 camera_pose);
    void update(double dt);
    Matrix3x4 get_camera_pose() const;
    Matrix3x4 get_view_transform() const;

private:
    Matrix3x4 camera_pose;
    bool camera_transform_changes_handedness{ false };
    Vector2 last_mouse_pos{ -1.f };
    Vector3 velocity;
};
