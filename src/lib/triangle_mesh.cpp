#include "std.h"
#include "lib/common.h"
#include "triangle_mesh.h"

void Triangle_Mesh::print_info() const {
    size_t mesh_size =
        vertices.size() * sizeof(Vector3) +
        normals.size() * sizeof(Vector3) +
        uvs.size() * sizeof(Vector2) +
        indices.size() * sizeof(int32_t);

    printf("[mesh]\n");
    printf("vertex count = %d\n", get_vertex_count());
    printf("triangle count = %d\n", get_triangle_count());
    printf("mesh size = %zdK\n", mesh_size / 1024);
    printf("\n");
}

static void convert_to_mesh_with_face_normals(Triangle_Mesh& mesh) {
    std::vector<Vector3> temp_positions(mesh.indices.size());
    std::vector<Vector2> temp_uvs(mesh.indices.size());

    mesh.normals.resize(mesh.indices.size());

    ASSERT(mesh.indices.size() % 3 == 0);
    for (int i = 0; i < (int)mesh.indices.size(); i += 3) {
        int ia = mesh.indices[i + 0];
        int ib = mesh.indices[i + 1];
        int ic = mesh.indices[i + 2];

        temp_positions[i + 0] = mesh.vertices[ia];
        temp_positions[i + 1] = mesh.vertices[ib];
        temp_positions[i + 2] = mesh.vertices[ic];

        Vector3 n = cross(mesh.vertices[ib] - mesh.vertices[ia], mesh.vertices[ic] - mesh.vertices[ia]).normalized();
        mesh.normals[i + 0] = n;
        mesh.normals[i + 1] = n;
        mesh.normals[i + 2] = n;

        temp_uvs[i + 0] = mesh.uvs[ia];
        temp_uvs[i + 1] = mesh.uvs[ib];
        temp_uvs[i + 2] = mesh.uvs[ic];
    }
    std::swap(mesh.vertices, temp_positions);
    std::swap(mesh.uvs, temp_uvs);
}

static void duplicate_vertices_due_to_crease_angle_threshold_and_init_normal_groups(std::vector<uint64_t>& normal_groups, float crease_angle, Triangle_Mesh& mesh) {
    normal_groups.resize(mesh.vertices.size());

    std::unordered_map<Vector3, std::vector<int>> vertex_faces;
    for (int i = 0, face = 0; i < (int)mesh.indices.size(); i += 3, face++) {
        vertex_faces[mesh.vertices[mesh.indices[i + 0]]].push_back(face);
        vertex_faces[mesh.vertices[mesh.indices[i + 1]]].push_back(face);
        vertex_faces[mesh.vertices[mesh.indices[i + 2]]].push_back(face);
    }

    auto get_face_normal = [&mesh](int face) {
        Vector3 a = mesh.vertices[mesh.indices[face*3 + 0]];
        Vector3 b = mesh.vertices[mesh.indices[face*3 + 1]];
        Vector3 c = mesh.vertices[mesh.indices[face*3 + 2]];
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
            if (int index = mesh.indices[face*3 + 0]; mesh.vertices[index] == pos)
                normal_groups[index] = mask_infos[0].mask;
            else if (int index = mesh.indices[face*3 + 1]; mesh.vertices[index] == pos)
                normal_groups[index] = mask_infos[0].mask;
            else 
            {
                ASSERT(mesh.vertices[mesh.indices[face*3 + 2]] == pos);
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
                int ia = mesh.indices[face*3 + 0];
                int ib = mesh.indices[face*3 + 1];
                int ic = mesh.indices[face*3 + 2];

                int v_index;
                int v_index_index;
                if (mesh.vertices[ia] == pos) {
                    v_index = ia;
                    v_index_index = face*3 + 0;
                } else if (mesh.vertices[ib] == pos) {
                    v_index = ib;
                    v_index_index = face*3 + 1;
                } else {
                    ASSERT(mesh.vertices[ic] == pos);
                    v_index = ic;
                    v_index_index = face*3 + 2;
                }

                bool new_vertex = true;
                for (int uv_index = 0; uv_index < uv_count; uv_index++) {
                    if (mesh.uvs[v_index] == uvs[uv_index]) {
                        new_vertex = false;
                        mesh.indices[v_index_index] = new_vertex_indices[uv_index];
                    }
                }
                if (new_vertex) {
                    ASSERT(uv_count < max_uvs);
                    uvs[uv_count] = mesh.uvs[v_index];
                    new_vertex_indices[uv_count] = (int)mesh.vertices.size();
                    mesh.vertices.push_back(mesh.vertices[v_index]);
                    mesh.normals.push_back(mesh.normals[v_index]);
                    mesh.uvs.push_back(mesh.uvs[v_index]);
                    mesh.indices[v_index_index] = new_vertex_indices[uv_count];
                    normal_groups.push_back(mask_infos[i].mask);
                    uv_count++;
                }
            }
        }
    }
}

static void adjust_normal_for_duplicated_vertices(const std::vector<uint64_t>& normal_groups, Triangle_Mesh& mesh) {
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
    // If vertex has no duplicates then store its position as 'index',
    // otherwise store multiple 'indices'.
    struct Siblings {
        int index = -1;
        std::vector<int> indices;
    };

    std::unordered_map<Vertex_Info, Siblings, Vertex_Info_Hasher> duplicated_vertices;
    for (int i = 0; i < (int)mesh.vertices.size(); i++) {
        Vertex_Info v_info = { mesh.vertices[i], normal_groups[i] };
        auto it = duplicated_vertices.find(v_info);
        if (it != duplicated_vertices.end()) {
            Siblings& s = it->second;
            if (s.index != -1) {
                s.indices.push_back(s.index);
                s.index = -1;
                
            }
            s.indices.push_back(i);
        }
        else {
            duplicated_vertices.insert({v_info, {i}});
        }
    }

    for (const auto& entry : duplicated_vertices) {
        const Siblings& siblings = entry.second;
        if (siblings.indices.empty()) {
            ASSERT(siblings.index != -1);
            continue;
        }
        Vector3 n;
        for (int i : siblings.indices) {
            n += mesh.normals[i];
        }
        for (int i : siblings.indices) {
            mesh.normals[i] = n;
        }
    }
}

void calculate_normals(const Normal_Calculation_Params& params, Triangle_Mesh& mesh) {
    // Calculate face normals if requested.
    if (params.face_normals) {
        convert_to_mesh_with_face_normals(mesh);
        return;
    }

    // Duplicate vertices due to crease angle if requested.
    std::vector<uint64_t> normal_groups;
    if (params.use_crease_angle != 0.f) {
        normal_groups.resize(mesh.vertices.size(), 0);
        duplicate_vertices_due_to_crease_angle_threshold_and_init_normal_groups(normal_groups, params.crease_angle, mesh);
    }

    for (Vector3& n : mesh.normals) {
        n = Vector3_Zero;
    }
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        int i0 = mesh.indices[i + 0];
        int i1 = mesh.indices[i + 1];
        int i2 = mesh.indices[i + 2];

        Vector3 a = mesh.vertices[i0];
        Vector3 b = mesh.vertices[i1];
        Vector3 c = mesh.vertices[i2];

        Vector3 scaled_n_a;
        Vector3 scaled_n_b;
        Vector3 scaled_n_c;

        if (params.averaging_mode == Normal_Averaging_Mode::angle) {
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
            ASSERT(params.averaging_mode == Normal_Averaging_Mode::area);
            scaled_n_a = cross(b - a, c - a);
            scaled_n_b = scaled_n_a;
            scaled_n_c = scaled_n_a;
        }

        mesh.normals[i0] += scaled_n_a;
        mesh.normals[i1] += scaled_n_b;
        mesh.normals[i2] += scaled_n_c;
    }

    if (params.detect_duplicated_vertices) {
        adjust_normal_for_duplicated_vertices(normal_groups, mesh);
    }

    for (Vector3& n : mesh.normals) {
        if (n == Vector3_Zero) {
            n = Vector3(0, 0, 1); // default value for degenerated triangle
        }
        n.normalize();
    }
}
