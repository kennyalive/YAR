#pragma once

#include "matrix.h"
#include "vector.h"
#include <vector>

struct Mesh_Vertex {
    Vector3 pos;
    Vector3 normal;
    Vector2 uv;
};

struct Mesh_Data {
    std::vector<Mesh_Vertex> vertices;
    std::vector<uint32_t> indices;
};

// Defines how face normals are averaged to compute the vertex normal.
enum class Normal_Average_Mode {
    area, // normals are averaged based on face area
    angle // normals are averaged based on angle between face edges
};

struct Mesh_Load_Params {
    Matrix3x4 transform = Matrix3x4::identity;

    // This is only used when model file does not provide normals.
    Normal_Average_Mode normal_average_mode = Normal_Average_Mode::area;

    // crease_angle - in radians. 0.f to disable detection of edges that should have a sharp crease
    float crease_angle = 0.f;

    // If set then normals are computed per face. Could be useful for debugging to visualise faces.
    // This will overwrite normmals that are provided by the model file.
    bool face_normals = false;

    bool invert_winding_order = false;
};

void compute_normals(Mesh_Data& mesh, Normal_Average_Mode normal_average_mode, float crease_angle);
