#pragma once

#include "lib/vector.h"
#include <vector>

struct Vertex {
    Vector3 pos;
    Vector3 normal;
    Vector2 uv;
};

struct Mesh_Data {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    Vector3 k_diffuse;
    Vector3 k_specular;
};

std::vector<Mesh_Data> load_obj(const std::string& obj_file, float additional_scale);
void compute_normals(const Vector3* vertex_positions, uint32_t vertex_count, uint32_t vertex_stride, const uint32_t* indices, uint32_t index_count, Vector3* normals);
