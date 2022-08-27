#pragma once

#include "bounding_box.h"
#include "matrix.h"

struct Triangle_Mesh {
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    std::vector<int32_t> indices;
    int alpha_texture_index = -1;

    int get_triangle_count() const
    {
        ASSERT(indices.size() % 3 == 0);
        return (int)indices.size() / 3;
    }

    int get_vertex_count() const
    {
        return (int)vertices.size();
    }

    void get_positions(uint32_t triangle_index, Vector3 p[3]) const
    {
        const int* pi = &indices[triangle_index * 3];
        p[0] = vertices[pi[0]];
        p[1] = vertices[pi[1]];
        p[2] = vertices[pi[2]];
    }

    Vector3 get_position(uint32_t triangle_index, const Vector3& barycentrics) const
    {
        Vector3 p[3];
        get_positions(triangle_index, p);
        return barycentric_interpolate(p, barycentrics);
    }

    void get_normals(uint32_t triangle_index, Vector3 n[3]) const
    {
        const int* pi = &indices[triangle_index * 3];
        n[0] = normals[pi[0]];
        n[1] = normals[pi[1]];
        n[2] = normals[pi[2]];
    }

    Vector3 get_normal(uint32_t triangle_index, const Vector3& barycentrics) const
    {
        Vector3 n[3];
        get_normals(triangle_index, n);
        return barycentric_interpolate(n, barycentrics);
    }

    void get_uvs(uint32_t triangle_index, Vector2 uv[3]) const
    {
        const int* pi = &indices[triangle_index * 3];
        uv[0] = uvs[pi[0]];
        uv[1] = uvs[pi[1]];
        uv[2] = uvs[pi[2]];
    }

    Vector2 get_uv(uint32_t triangle_index, const Vector3& barycentrics) const
    {
        Vector2 uv[3];
        get_uvs(triangle_index, uv);
        return barycentric_interpolate(uv, barycentrics);
    }

    Bounding_Box get_triangle_bounds(uint32_t triangle_index) const
    {
        const int* pi = &indices[triangle_index * 3];
        auto bounds = Bounding_Box(vertices[pi[0]]);
        bounds.add_point(vertices[pi[1]]);
        bounds.add_point(vertices[pi[2]]);
        return bounds;
    }

    Bounding_Box get_bounds() const
    {
        Bounding_Box bounds;
        for (int i = 0; i < get_triangle_count(); i++) {
            bounds = Bounding_Box::compute_union(bounds, get_triangle_bounds(i));
        }
        return bounds;
    }

    void remove_degenerate_triangles();
    void print_info() const;
};

// Defines how face normals are averaged to compute the vertex normal.
enum class Normal_Averaging_Mode {
    area, // normals are averaged based on face area
    angle // normals are averaged based on angle between face edges
};

struct Normal_Calculation_Params {
    Normal_Averaging_Mode averaging_mode = Normal_Averaging_Mode::area;

    // If different vertices correspond to the same position and this flag is set then
    // consider it's the same vertex for the purposes of normal calculation.
    // If use_crease_angle flag is set then vertices with the same position could still
    // be considered as separated.
    bool detect_duplicated_vertices = false;

    // Detection of edges that should have a sharp crease.
    bool use_crease_angle = false;
    float crease_angle = 0.f; // in radians

    // If set then normals are computed per face. Could be useful for debugging to visualise faces.
    bool face_normals = false;
};

void calculate_normals(const Normal_Calculation_Params& params, Triangle_Mesh& mesh);

struct Triangle_Mesh_Load_Params {
    Matrix3x4 transform = Matrix3x4::identity;

    // Forces normal calculation (overrides model's normals if they are provided).
    // By default, normals are calculated only if they are not provided in the source data.
    bool force_normal_calculation = false;

    Normal_Calculation_Params normal_calculation_params;

    bool invert_winding_order = false;
};
