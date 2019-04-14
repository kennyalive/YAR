#pragma once

#include "light.h"
#include "material.h"
#include "lib/bounding_box.h"
#include "lib/mesh.h"
#include <vector>

struct Triangle_Mesh {
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    std::vector<uint32_t> indices;
    Material_Handle material = Null_Material;
    Light_Handle area_light = Null_Light;

    static Triangle_Mesh from_mesh_data(const Mesh_Data& mesh_data, Material_Handle material);
    static Triangle_Mesh from_diffuse_rectangular_light(const Diffuse_Rectangular_Light& light, int light_index);

    uint32_t get_triangle_count() const {
        ASSERT(indices.size() % 3 == 0);
        return static_cast<uint32_t>(indices.size() / 3);
    }

    uint32_t get_vertex_count() const {
        return static_cast<uint32_t>(vertices.size());
    }

    void get_triangle(uint32_t triangle_index, Vector3& p0, Vector3& p1, Vector3& p2) const {
        const uint32_t* pi = &indices[triangle_index * 3];
        p0 = vertices[pi[0]];
        p1 = vertices[pi[1]];
        p2 = vertices[pi[2]];
    }

    Vector3 get_normal(uint32_t triangle_index, float b1, float b2) const {
        const uint32_t* pi = &indices[triangle_index * 3];
        Vector3 n0 = normals[pi[0]];
        Vector3 n1 = normals[pi[1]];
        Vector3 n2 = normals[pi[2]];

        return ((1.f - b1 - b2)*n0 + b1 * n1 + b2 * n2).normalized();
    }

    Bounding_Box get_triangle_bounds(uint32_t triangle_index) const;
    Bounding_Box get_bounds() const;
    void print_info() const;
};
