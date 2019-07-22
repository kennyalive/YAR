#pragma once

#include "bounding_box.h"
#include "matrix.h"

struct Triangle_Mesh {
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    std::vector<int32_t> indices;

    int get_triangle_count() const {
        ASSERT(indices.size() % 3 == 0);
        return static_cast<int>(indices.size() / 3);
    }

    int get_vertex_count() const {
        return static_cast<int>(vertices.size());
    }

    void get_triangle(int triangle_index, Vector3& p0, Vector3& p1, Vector3& p2) const {
        const int* pi = &indices[triangle_index * 3];
        p0 = vertices[pi[0]];
        p1 = vertices[pi[1]];
        p2 = vertices[pi[2]];
    }

    Vector3 get_normal(int triangle_index, float b1, float b2) const {
        const int* pi = &indices[triangle_index * 3];
        Vector3 n0 = normals[pi[0]];
        Vector3 n1 = normals[pi[1]];
        Vector3 n2 = normals[pi[2]];
        return ((1.f - b1 - b2)*n0 + b1 * n1 + b2 * n2).normalized();
    }
    
    Vector2 get_uv(int triangle_index, float b1, float b2) const {
        const int* pi = &indices[triangle_index * 3];
        Vector2 uv0 = uvs[pi[0]];
        Vector2 uv1 = uvs[pi[1]];
        Vector2 uv2 = uvs[pi[2]];
        return (1.f - b1 - b2)*uv0 + b1*uv1 + b2*uv2;
    }

    Bounding_Box get_triangle_bounds(int triangle_index) const {
        const int* pi = &indices[triangle_index * 3];
        auto bounds = Bounding_Box(vertices[pi[0]]);
        bounds.add_point(vertices[pi[1]]);
        bounds.add_point(vertices[pi[2]]);
        return bounds;
    }

    Bounding_Box get_bounds() const {
        Bounding_Box bounds;
        for (int i = 0; i < get_triangle_count(); i++) {
            bounds = Bounding_Box::compute_union(bounds, get_triangle_bounds(i));
        }
        return bounds;
    }

    void print_info() const;
};

// Defines how face normals are averaged to compute the vertex normal.
enum class Normal_Average_Mode {
    area, // normals are averaged based on face area
    angle // normals are averaged based on angle between face edges
};

struct Triangle_Mesh_Load_Params {
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

void compute_normals(Triangle_Mesh& mesh, Normal_Average_Mode normal_average_mode, float crease_angle);

