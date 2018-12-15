#pragma once

#include "matrix.h"
#include "vector.h"
#include <vector>

struct Vertex {
    Vector3 pos;
    Vector3 normal;
    Vector2 uv;

    bool operator==(const Vertex& other) const {
        return pos == other.pos && normal == other.normal && uv == other.uv;
    }
};

struct Mesh_Data {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    Vector3 k_diffuse;
    Vector3 k_specular;
};

void compute_normals(
    const Vector3* vertex_positions,
    const uint64_t* vertex_normal_groups,
    uint32_t vertex_count,
    uint32_t vertex_stride,
    const uint32_t* indices,
    uint32_t index_count,
    Vector3* normals);

std::vector<Mesh_Data> load_obj(const std::string& obj_file, const Matrix3x4& transform = Matrix3x4::identity);
