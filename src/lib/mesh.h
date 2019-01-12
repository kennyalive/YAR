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
    Vector3 k_diffuse;
    Vector3 k_specular;
};

// Defines how face normals are averaged to compute the vertex normal.
enum class Normal_Average_Mode {
    angle, // normals are averaged based on angle between face edges
    area // normals are averaged based on face area
};

struct Mesh_Load_Params {
    Matrix3x4 transform = Matrix3x4::identity;

    // This is only used when model file does not provide normals.
    Normal_Average_Mode normal_average_mode = Normal_Average_Mode::angle;

    // If set then normals are computed per face. Could be useful for debugging to visualise faces.
    // This will overwrite normmals that are provided by the model file.
    bool face_normals = false;
};

void compute_normals(
    const Vector3* vertex_positions, // #vertex_count
    const uint64_t* normal_groups, // #vertex_count
    uint32_t vertex_count,
    uint32_t vertex_stride,
    const uint32_t* indices, // #index_count
    uint32_t index_count,
    Normal_Average_Mode normal_average_mode,
    Vector3* normals // #vertex_count
);

std::vector<Mesh_Data> load_obj(const std::string& obj_file, const Mesh_Load_Params params = Mesh_Load_Params{});
