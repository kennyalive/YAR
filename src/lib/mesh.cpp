#include "std.h"
#include "mesh.h"

#include <unordered_map>

static void duplicate_vertices_due_to_crease_angle_threshold(Mesh_Data& mesh, std::vector<uint64_t>& normal_groups, float crease_angle) {
    normal_groups.resize(mesh.vertices.size());

    std::unordered_map<Vector3, std::vector<int>> vertex_faces;
    for (int i = 0, face = 0; i < (int)mesh.indices.size(); i += 3, face++) {
        vertex_faces[mesh.vertices[mesh.indices[i + 0]].pos].push_back(face);
        vertex_faces[mesh.vertices[mesh.indices[i + 1]].pos].push_back(face);
        vertex_faces[mesh.vertices[mesh.indices[i + 2]].pos].push_back(face);
    }

    auto get_face_normal = [&mesh](int face) {
        Vector3 a = mesh.vertices[mesh.indices[face*3 + 0]].pos;
        Vector3 b = mesh.vertices[mesh.indices[face*3 + 1]].pos;
        Vector3 c = mesh.vertices[mesh.indices[face*3 + 2]].pos;
        return cross(b - a, c - a).normalized();
    };

    struct Mask_Info {
        uint64_t mask;
        std::vector<int> faces;
    };
    std::vector<Mask_Info> mask_infos; // mask infos are re-initialized for each vertex
    int mask_info_count;

    const float crease_angle_cos = std::cos(crease_angle);

    for (const std::pair<Vector3, std::vector<int>>& entry : vertex_faces) {
        const Vector3 pos = entry.first;
        const std::vector<int>& faces = entry.second;
        ASSERT(faces.size() <= 64);

        // reset mask infos
        for (auto& info : mask_infos) {
            info.mask = 0;
            info.faces.resize(0);
        }
        mask_info_count = 0;

        // Compute mask for each face. Each bit in a mask defines if corresponding face
        // forms an angle with current face that is less than the crease angle.
        uint64_t masks[64];
        for (size_t i = 0; i < faces.size(); i++) {
            masks[i] = 1ull << i;
        }
        for (size_t i = 0; i < faces.size()-1; i++) {
            for (size_t k = i + 1; k < faces.size(); k++) {
                int face_a = faces[i];
                int face_b = faces[k];

                Vector3 normal_a = get_face_normal(face_a);
                Vector3 normal_b = get_face_normal(face_b);

                if (dot(normal_a, normal_b) > crease_angle_cos) {
                    masks[i] |= 1ull << k;
                    masks[k] |= 1ull << i;
                }
            }
        }

        // sort faces by mask
        for (size_t i = 0; i < faces.size(); i++) {
            int k = 0;
            for (; k < mask_info_count; k++)
                if (mask_infos[k].mask == masks[i])
                    break;

            if (k == mask_info_count) {
                if (mask_infos.size() == mask_info_count) {
                    mask_infos.push_back(Mask_Info{});
                }
                mask_infos[mask_info_count++].mask = masks[i];
            }
            mask_infos[k].faces.push_back(faces[i]);
        }

        // update normal_groups for the first mask group
        for (int face : mask_infos[0].faces) {
            if (uint32_t index = mesh.indices[face*3 + 0]; mesh.vertices[index].pos == pos)
                normal_groups[index] = mask_infos[0].mask;
            else if (uint32_t index = mesh.indices[face*3 + 1]; mesh.vertices[index].pos == pos)
                normal_groups[index] = mask_infos[0].mask;
            else 
            {
                ASSERT(mesh.vertices[mesh.indices[face*3 + 2]].pos == pos);
                normal_groups[mesh.indices[face*3 + 2]] = mask_infos[0].mask;
            }
        }

        // Faces from the first mask group could use original vertices,
        // for other mask groups we need to duplicate vertices in order to have unique normals.
        for (int i = 1; i < mask_info_count; i++) {
            const std::vector<int>& grouped_faces = mask_infos[i].faces;

            const int max_uvs = 4;
            Vector2 uvs[max_uvs];
            int new_vertex_indices[max_uvs];
            int uv_count = 0;

            for (int face : grouped_faces) {
                Mesh_Vertex a = mesh.vertices[mesh.indices[face*3 + 0]];
                Mesh_Vertex b = mesh.vertices[mesh.indices[face*3 + 1]];
                Mesh_Vertex c = mesh.vertices[mesh.indices[face*3 + 2]];

                Mesh_Vertex v;
                int v_index_index;
                if (a.pos == pos) {
                    v = a;
                    v_index_index = face*3 + 0;
                } else if (b.pos == pos) {
                    v = b;
                    v_index_index = face*3 + 1;
                } else {
                    ASSERT(c.pos == pos);
                    v = c;
                    v_index_index = face*3 + 2;
                }

                bool new_vertex = true;
                for (int uv_index = 0; uv_index < uv_count; uv_index++) {
                    if (v.uv == uvs[uv_index]) {
                        new_vertex = false;
                        mesh.indices[v_index_index] = new_vertex_indices[uv_index];
                    }
                }
                if (new_vertex) {
                    ASSERT(uv_count < max_uvs);
                    uvs[uv_count] = v.uv;
                    new_vertex_indices[uv_count] = (int)mesh.vertices.size();
                    mesh.vertices.push_back(v);
                    mesh.indices[v_index_index] = new_vertex_indices[uv_count];
                    normal_groups.push_back(mask_infos[i].mask);
                    uv_count++;
                }
            }
        }
    }
}

void compute_normals(Mesh_Data& mesh, Normal_Average_Mode normal_average_mode, float crease_angle) {
    struct Vertex_Info {
        Vector3 pos;
        uint64_t normal_group;

        bool operator==(const Vertex_Info& other) const {
            return pos == other.pos && normal_group == other.normal_group;
        }
    };

    struct Vertex_Info_Hasher {
        size_t operator()(const Vertex_Info& v) const {
            size_t hash = 0;
            hash_combine(hash, v.pos);
            hash_combine(hash, v.normal_group);
            return hash;
        }
    };

    std::vector<uint64_t> normal_groups(mesh.vertices.size(), 0);

    if (crease_angle != 0.f)
        duplicate_vertices_due_to_crease_angle_threshold(mesh, normal_groups, crease_angle);

    // Vertices with the same position and normal group but different texture coordinates.
    std::unordered_map<Vertex_Info, std::vector<uint32_t>, Vertex_Info_Hasher> duplicated_vertices;

    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        Vertex_Info v_info = { mesh.vertices[i].pos, normal_groups[i] };
        duplicated_vertices[v_info].push_back((uint32_t)i);
    }

    std::vector<bool> has_duplicates(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        Vertex_Info v_info = { mesh.vertices[i].pos, normal_groups[i] };
        size_t vertex_count = duplicated_vertices[v_info].size();
        ASSERT(vertex_count > 0);
        has_duplicates[i] = vertex_count > 1;
    }

    for (Mesh_Vertex& v : mesh.vertices)
        v.normal = Vector3_Zero;

    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        uint32_t i0 = mesh.indices[i + 0];
        uint32_t i1 = mesh.indices[i + 1];
        uint32_t i2 = mesh.indices[i + 2];

        Vector3 a = mesh.vertices[i0].pos;
        Vector3 b = mesh.vertices[i1].pos;
        Vector3 c = mesh.vertices[i2].pos;

        Vector3 scaled_n_a;
        Vector3 scaled_n_b;
        Vector3 scaled_n_c;

        if (normal_average_mode == Normal_Average_Mode::angle) {
            Vector3 d1 = b - a;
            Vector3 d2 = c - a;
            float angle = std::acos(std::clamp(dot(d1.normalized(), d2.normalized()), -1.f, 1.f));
            scaled_n_a = cross(d1, d2).normalized() * angle;

            d1 = c - b;
            d2 = a - b;
            angle = std::acos(std::clamp(dot(d1.normalized(), d2.normalized()), -1.f, 1.f));
            scaled_n_b = cross(d1, d2).normalized() * angle;

            d1 = a - c;
            d2 = b - c;
            angle = std::acos(std::clamp(dot(d1.normalized(), d2.normalized()), -1.f, 1.f));
            scaled_n_c = cross(d1, d2).normalized() * angle;
        } else {
            ASSERT(normal_average_mode == Normal_Average_Mode::area);
            scaled_n_a = cross(b - a, c - a);
            scaled_n_b = scaled_n_a;
            scaled_n_c = scaled_n_a;
        }

        if (has_duplicates[i0]) {
            for (uint32_t vi : duplicated_vertices[{a, normal_groups[i0]}])
                mesh.vertices[vi].normal += scaled_n_a;
        } else {
            mesh.vertices[i0].normal += scaled_n_a;
        }

        if (has_duplicates[i1]) {
            for (uint32_t vi : duplicated_vertices[{b, normal_groups[i1]}])
                mesh.vertices[vi].normal += scaled_n_b;
        } else {
            mesh.vertices[i1].normal += scaled_n_b;
        }

        if (has_duplicates[i2]) {
            for (uint32_t vi : duplicated_vertices[{c, normal_groups[i2]}])
                mesh.vertices[vi].normal += scaled_n_c;
        } else {
            mesh.vertices[i2].normal += scaled_n_c;
        }
    }

    for (Mesh_Vertex& v : mesh.vertices) {
        if (v.normal == Vector3_Zero) {
            v.normal = Vector3(0, 0, 1); // default value for degenerated triangle
        }
        v.normal.normalize();
    }
}
