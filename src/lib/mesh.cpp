#include "bounding_box.h"
#include "mesh.h"

#include <algorithm>
#include <unordered_map>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

static void duplicate_vertices_due_to_crease_angle_threshold(Mesh_Data& mesh, std::vector<uint64_t>& vertex_normal_groups) {
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

    struct Normal_Group_Info {
        uint64_t normal_group;
        std::vector<int> faces;
    };
    std::vector<Normal_Group_Info> normal_group_infos;
    int normal_group_count;

    vertex_normal_groups.resize(mesh.vertices.size());

    for (const std::pair<Vector3, std::vector<int>>& entry : vertex_faces) {
        const Vector3 pos = entry.first;
        const std::vector<int>& faces = entry.second;
        assert(faces.size() <= 64);

        for (auto& info : normal_group_infos) {
            info.normal_group = 0;
            info.faces.resize(0);
        }
        normal_group_count = 0;

        // compute normal group mask for each face
        uint64_t masks[64];
        for (size_t i = 0; i < faces.size(); i++) {
            masks[i] = 1ull << i;
        }
        for (size_t i = 0; i < faces.size()-1; i++) {
            for (size_t k = 1; k < faces.size(); k++) {
                int face_a = faces[i];
                int face_b = faces[k];

                Vector3 normal_a = get_face_normal(face_a);
                Vector3 normal_b = get_face_normal(face_b);

                if (dot(normal_a, normal_b) > 0.5f /*60deg*/) {
                    masks[i] |= 1ull << k;
                    masks[k] |= 1ull << i;
                }
            }
        }

        // sort faces by normal group
        for (size_t i = 0; i < faces.size(); i++) {
            int k = 0;
            for (k = 0; k < normal_group_count; k++) {
                if (normal_group_infos[k].normal_group == masks[i])
                    break;
            }
            if (k == normal_group_count) {
                if (normal_group_infos.size() == normal_group_count) {
                    normal_group_infos.push_back(Normal_Group_Info{});
                }
                normal_group_infos[normal_group_count++].normal_group = masks[i];
            }
            normal_group_infos[k].faces.push_back(faces[i]);
        }

        // update vertex_normal_groups for the first normal group
        for (int face : normal_group_infos[0].faces) {
            vertex_normal_groups[mesh.indices[face*3 + 0]] = normal_group_infos[0].normal_group;
            vertex_normal_groups[mesh.indices[face*3 + 1]] = normal_group_infos[0].normal_group;
            vertex_normal_groups[mesh.indices[face*3 + 2]] = normal_group_infos[0].normal_group;
        }

        // Faces from the first normal group could use original vertices,
        // for other normal groups we need to duplicate vertices in order to have unique normals.
        for (int i = 1; i < normal_group_count; i++) {
            const std::vector<int>& grouped_faces = normal_group_infos[i].faces;

            const int max_uvs = 4;
            Vector2 uvs[max_uvs];
            int new_vertex_indices[max_uvs];
            int uv_count = 0;

            for (int face : grouped_faces) {
                Vertex a = mesh.vertices[mesh.indices[face*3 + 0]];
                Vertex b = mesh.vertices[mesh.indices[face*3 + 1]];
                Vertex c = mesh.vertices[mesh.indices[face*3 + 2]];

                Vertex v;
                int v_index_index;
                if (a.pos == pos) {
                    v = a;
                    v_index_index = face*3 + 0;
                } else if (b.pos == pos) {
                    v = b;
                    v_index_index = face*3 + 1;
                } else {
                    v = c;
                    assert(c.pos == pos);
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
                    assert(uv_count < max_uvs);
                    uvs[uv_count] = v.uv;
                    new_vertex_indices[uv_count] = (int)mesh.vertices.size();
                    mesh.vertices.push_back(v);
                    mesh.indices[v_index_index] = new_vertex_indices[uv_count];
                    vertex_normal_groups.push_back(normal_group_infos[i].normal_group);
                    uv_count++;
                }
            }
        }
    }
}

void compute_normals(
    const Vector3* vertex_positions,
    const uint64_t* vertex_normal_groups,
    uint32_t vertex_count,
    uint32_t vertex_stride,
    const uint32_t* indices,
    uint32_t index_count,
    Vector3* normals)
{
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

    // Vertices with the same position and normal group but different texture coordinates
    std::unordered_map<Vertex_Info, std::vector<uint32_t>, Vertex_Info_Hasher> duplicated_vertices;

    for (uint32_t i = 0; i < vertex_count; i++) {
        Vector3 pos = index_array_with_stride(vertex_positions, vertex_stride, i);
        Vertex_Info v_info = { pos, vertex_normal_groups[i] };
        duplicated_vertices[v_info].push_back(i);
    }

    std::vector<bool> has_duplicates(vertex_count);
    for (uint32_t i = 0; i < vertex_count; i++) {
        Vector3 pos = index_array_with_stride(vertex_positions, vertex_stride, i);
        Vertex_Info v_info = { pos, vertex_normal_groups[i] };
        uint32_t vertex_count = (uint32_t)duplicated_vertices[v_info].size();
        assert(vertex_count > 0);
        has_duplicates[i] = vertex_count > 1;
    }

    for (uint32_t i = 0; i < vertex_count; i++)
        index_array_with_stride(normals, vertex_stride, i) = Vector3_Zero;

    for (uint32_t i = 0; i < index_count; i += 3) {
        uint32_t i0 = indices[i + 0];
        uint32_t i1 = indices[i + 1];
        uint32_t i2 = indices[i + 2];

        Vector3 a = index_array_with_stride(vertex_positions, vertex_stride, i0);
        Vector3 b = index_array_with_stride(vertex_positions, vertex_stride, i1);
        Vector3 c = index_array_with_stride(vertex_positions, vertex_stride, i2);

        Vector3 d1 = b - a;
        Vector3 d2 = c - a;
        Vector3 n_scaled_by_area = cross(d1, d2);

        if (has_duplicates[i0]) {
            for (uint32_t vi : duplicated_vertices[{a, vertex_normal_groups[i0]}])
                index_array_with_stride(normals, vertex_stride, vi) += n_scaled_by_area;
        } else {
            index_array_with_stride(normals, vertex_stride, i0) += n_scaled_by_area;
        }

        if (has_duplicates[i1]) {
            for (uint32_t vi : duplicated_vertices[{b, vertex_normal_groups[i1]}])
                index_array_with_stride(normals, vertex_stride, vi) += n_scaled_by_area;
        } else {
            index_array_with_stride(normals, vertex_stride, i1) += n_scaled_by_area;
        }

        if (has_duplicates[i2]) {
            for (uint32_t vi : duplicated_vertices[{c, vertex_normal_groups[i2]}])
                index_array_with_stride(normals, vertex_stride, vi) += n_scaled_by_area;
        } else {
            index_array_with_stride(normals, vertex_stride, i2) += n_scaled_by_area;
        }
    }

    for (uint32_t i = 0; i < vertex_count; i++) {
        Vector3& n = index_array_with_stride(normals, vertex_stride, i);
        assert(n.length() > 1e-6f);
        n.normalize();
    }
}

std::vector<Mesh_Data> load_obj(const std::string& obj_file, float additional_scale)
{
    struct Vertex_Hasher {
        size_t operator()(const Vertex& v) const {
            size_t hash = 0;
            hash_combine(hash, v.pos);
            hash_combine(hash, v.normal);
            hash_combine(hash, v.uv);
            return hash;
        }
    };

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;

    const std::string obj_path = get_resource_path(obj_file).c_str();
    const std::string mtl_dir = get_directory(obj_path);

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, obj_path.c_str(), mtl_dir.c_str()))
        error("failed to load obj model: " + obj_file);

    std::vector<Mesh_Data> meshes(shapes.size());
    Bounding_Box bounds;

    for (size_t i = 0; i < shapes.size(); i++) {
        tinyobj::shape_t& shape = shapes[i];
        Mesh_Data& mesh = meshes[i];

        std::unordered_map<Vertex, std::size_t, Vertex_Hasher> unique_vertices;
        bool has_normals = true;

        for (const auto& index : shape.mesh.indices) {
            Vertex vertex;

            vertex.pos = {
                attrib.vertices[3 * index.vertex_index + 0],
                attrib.vertices[3 * index.vertex_index + 1],
                attrib.vertices[3 * index.vertex_index + 2]
            };

            if (!attrib.normals.empty() && index.normal_index != -1) {
                assert(index.normal_index != -1);
                vertex.normal = {
                    attrib.normals[3 * index.normal_index + 0],
                    attrib.normals[3 * index.normal_index + 1],
                    attrib.normals[3 * index.normal_index + 2],
                };
            } else {
                vertex.normal = Vector3_Zero;
                has_normals = false;
            }

            if (!attrib.texcoords.empty() && index.texcoord_index != -1) {
                vertex.uv = {
                    attrib.texcoords[2 * index.texcoord_index + 0],
                    1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
                };
            } else {
                vertex.uv = Vector2_Zero;
            }

            if (unique_vertices.count(vertex) == 0) {
                unique_vertices[vertex] = mesh.vertices.size();
                mesh.vertices.push_back(vertex);
                bounds.add_point(vertex.pos);
            }
            mesh.indices.push_back((uint32_t)unique_vertices[vertex]);
        }

        if (!has_normals) {
            std::vector<uint64_t> vertex_normal_groups;
            duplicate_vertices_due_to_crease_angle_threshold(mesh, vertex_normal_groups);
            assert(vertex_normal_groups.size() == mesh.vertices.size());
            compute_normals(&mesh.vertices[0].pos, vertex_normal_groups.data(), (int)mesh.vertices.size(), (int)sizeof(Vertex), mesh.indices.data(), (int)mesh.indices.size(), &mesh.vertices[0].normal);
        }

        if (!shape.mesh.material_ids.empty() && shape.mesh.material_ids[0] != -1) {
            for (int face_material_id : shape.mesh.material_ids)
                assert(face_material_id == shape.mesh.material_ids[0]);

            const tinyobj::material_t& material = materials[shape.mesh.material_ids[0]];
            mesh.k_diffuse = Vector3(material.diffuse[0], material.diffuse[1], material.diffuse[2]);
            mesh.k_specular = Vector3(material.specular[0], material.specular[1], material.specular[2]);
        }
    }

    for (Mesh_Data& mesh : meshes) {
        for (Vertex& v : mesh.vertices) {
            v.pos *= 0.003f;
        }
    }
    return meshes;
}
