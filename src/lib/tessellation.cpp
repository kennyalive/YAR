#include "std.h"
#include "common.h"

#include "tessellation.h"

Triangle_Mesh create_cube_mesh(float s) {
    Triangle_Mesh mesh;
    mesh.vertices = {
        {-s, -s, -s},
        { s, -s, -s},
        { s,  s, -s},
        {-s,  s, -s},
        {-s, -s,  s},
        { s, -s,  s},
        { s,  s,  s},
        {-s,  s,  s},
    };
    mesh.indices = {
        0, 1, 2,
        2, 3, 0,
        0, 1, 5,
        5, 4, 0,
        1, 2, 6,
        6, 5, 1,
        2, 3, 7,
        7, 6, 2,
        3, 0, 4,
        4, 7, 3,
        4, 5, 6,
        6, 7, 4
    };
    Normal_Calculation_Params params;
    params.use_crease_angle = true;
    params.crease_angle = radians(5.f);
    calculate_normals(params, mesh);
    return mesh;
}

// Creates sphere geometry by building a geodesic grid. The grid is the result of
// subdivision of icosahendron's faces into 4 triangles at each subdivision step.
// Icosahedron's vertex coordinates and connectivity information is from:
// https://www.geometrictools.com/Documentation/PlatonicSolids.pdf
Triangle_Mesh create_sphere_mesh(float radius, int subdivision_level, bool texture_v_is_zero_at_bottom) {
    const float t = (1.f + std::sqrt(5.f)) / 2.f;
    const float s_inv = 1.f / std::sqrt(1.f + t*t);

    std::vector<Vector3> vertices = {
        { t,  1,  0},
        {-t,  1,  0},
        { t, -1,  0},
        {-t, -1,  0},
        { 1,  0,  t},
        { 1,  0, -t},
        {-1,  0,  t},
        {-1,  0, -t},
        { 0,  t,  1},
        { 0, -t,  1},
        { 0,  t, -1},
        { 0, -t, -1},
    };
    for (Vector3& v : vertices) v *= s_inv;

    std::vector<int32_t> indices = {
        0, 8, 4,
        1, 10, 7,
        2, 9, 11,
        7, 3, 1,
        0, 5, 10,
        3, 9, 6,
        3, 11, 9,
        8, 6, 4,
        2, 4, 9,
        3, 7, 11,
        4, 2, 0,
        9, 4, 6,
        2, 11, 5,
        0, 10, 8,
        5, 0, 2,
        10, 5, 7,
        1, 6, 8,
        1, 8, 10,
        6, 1, 3,
        11, 7 ,5
    };

    std::vector<Vector3> next_subdiv_vertices;
    std::vector<int32_t> next_subdiv_indices;
    std::unordered_map<Vector3, int32_t> position_to_index; // removes duplicated positions

    auto add_vertex = [&position_to_index, &next_subdiv_vertices](const Vector3& v) {
        auto it = position_to_index.find(v);
        if (it != position_to_index.end()) 
            return it->second;

        int32_t index = (int32_t)next_subdiv_vertices.size();
        next_subdiv_vertices.push_back(v);
        position_to_index.insert(std::make_pair(v, index));
        return index;
    };

    for (int i = 0; i < subdivision_level; i++) {
        for (int k = 0; k < indices.size(); k+= 3) {
            Vector3 v0 = vertices[indices[k + 0]];
            Vector3 v1 = vertices[indices[k + 1]];
            Vector3 v2 = vertices[indices[k + 2]];

            Vector3 v01 = (v0 + v1).normalized();
            Vector3 v12 = (v1 + v2).normalized();
            Vector3 v02 = (v0 + v2).normalized();

            int32_t i0 = add_vertex(v0);
            int32_t i1 = add_vertex(v1);
            int32_t i2 = add_vertex(v2);
            int32_t i3 = add_vertex(v01);
            int32_t i4 = add_vertex(v12);
            int32_t i5 = add_vertex(v02);

            // triangle 0
            next_subdiv_indices.push_back(i0);
            next_subdiv_indices.push_back(i3);
            next_subdiv_indices.push_back(i5);

            // triangle 1
            next_subdiv_indices.push_back(i1);
            next_subdiv_indices.push_back(i4);
            next_subdiv_indices.push_back(i3);

            // triangle 2
            next_subdiv_indices.push_back(i2);
            next_subdiv_indices.push_back(i5);
            next_subdiv_indices.push_back(i4);

            // triangle 3
            next_subdiv_indices.push_back(i3);
            next_subdiv_indices.push_back(i4);
            next_subdiv_indices.push_back(i5);
        }
        vertices.swap(next_subdiv_vertices);
        indices.swap(next_subdiv_indices);
        next_subdiv_vertices.resize(0);
        next_subdiv_indices.resize(0);
        position_to_index.clear();
    }

    std::vector<Vector2> uvs(vertices.size());
    for (int i = 0; i < (int)vertices.size(); i++) {
        Vector3 p = vertices[i];

        float cos_theta = std::clamp(texture_v_is_zero_at_bottom ? -p.z : p.z, -1.f, 1.f);
        float v = std::clamp(std::acos(cos_theta) / Pi, 0.f, One_Minus_Epsilon);

        float phi = std::atan2(p.y, p.x);
        if (phi < 0)
            phi += Pi2;
        float u = std::clamp(phi / Pi2, 0.f, One_Minus_Epsilon);

        uvs[i] = Vector2(u, v);
    }

    for (Vector3& v : vertices) {
        v *= radius;
    }

    Triangle_Mesh mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices = std::move(indices);
    mesh.uvs = std::move(uvs);

    calculate_normals(Normal_Calculation_Params{}, mesh);
    return mesh;
}
