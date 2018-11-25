#include "mesh.h"
#include "lib/bounding_box.h"

#include <algorithm>
#include <unordered_map>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

namespace std {
template<> struct hash<Vertex> {
    size_t operator()(Vertex const& v) const {
        size_t hash = 0;
        hash_combine(hash, v.pos);
        hash_combine(hash, v.normal);
        hash_combine(hash, v.uv);
        return hash;
    }
};
}

static inline bool operator==(const Vertex& v1, const Vertex& v2) {
    return v1.pos == v2.pos && v1.normal == v2.normal && v1.uv == v2.uv;
}

std::vector<Mesh_Data> load_obj(const std::string& obj_file, float additional_scale) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;

    std::string obj_path = get_resource_path(obj_file).c_str();
    std::string mtl_dir = get_directory(obj_path);

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, obj_path.c_str(), mtl_dir.c_str()))
        error("failed to load obj model: " + obj_file);

    std::vector<Mesh_Data> meshes(shapes.size());
    Bounding_Box bounds;

    for (size_t i = 0; i < shapes.size(); i++) {
        tinyobj::shape_t& shape = shapes[i];
        Mesh_Data& mesh = meshes[i];

        std::unordered_map<Vertex, std::size_t> unique_vertices;
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

        if (!has_normals)
            compute_normals(&mesh.vertices[0].pos, (int)mesh.vertices.size(), (int)sizeof(Vertex), mesh.indices.data(), (int)mesh.indices.size(), &mesh.vertices[0].normal);

        if (!shape.mesh.material_ids.empty() && shape.mesh.material_ids[0] != -1) {
            for (int face_material_id : shape.mesh.material_ids)
                assert(face_material_id == shape.mesh.material_ids[0]);

            const tinyobj::material_t& material = materials[shape.mesh.material_ids[0]];
            mesh.k_diffuse = Vector3(material.diffuse[0], material.diffuse[1], material.diffuse[2]);
            mesh.k_specular = Vector3(material.specular[0], material.specular[1], material.specular[2]);
        }
    }

    // scale and center the mesh
    Vector3 diag = bounds.max_p - bounds.min_p;
    float max_size = std::max(diag.x, std::max(diag.y, diag.z));
    float scale = (2.f / max_size) * additional_scale;

    Vector3 center = (bounds.max_p + bounds.min_p) * 0.5f;
    for (Mesh_Data& mesh : meshes) {
        for (Vertex& v : mesh.vertices) {
            v.pos -= center;
            v.pos *= scale;
        }
    }
   
    return meshes;
}

void compute_normals(const Vector3* vertex_positions, uint32_t vertex_count, uint32_t vertex_stride, const uint32_t* indices, uint32_t index_count, Vector3* normals) {
    std::unordered_map<Vector3, std::vector<uint32_t>> duplicated_vertices; // due to different texture coordinates
    for (uint32_t i = 0; i < vertex_count; i++) {
        const Vector3& pos = index_array_with_stride(vertex_positions, vertex_stride, i);
        duplicated_vertices[pos].push_back(i);
    }

    std::vector<bool> has_duplicates(vertex_count);
    for (uint32_t i = 0; i < vertex_count; i++) {
        const Vector3& pos = index_array_with_stride(vertex_positions, vertex_stride, i);
        uint32_t vertex_count = (uint32_t)duplicated_vertices[pos].size();
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
        assert(d1.length() > 1e-6f);
        Vector3 d2 = c - a;
        assert(d2.length() > 1e-6f);

        Vector3 n = cross(d1, d2).normalized();

        if (has_duplicates[i0]) {
            for (uint32_t vi : duplicated_vertices[a])
                index_array_with_stride(normals, vertex_stride, vi) += n;
        } else {
            index_array_with_stride(normals, vertex_stride, i0) += n;
        }

        if (has_duplicates[i1]) {
            for (uint32_t vi : duplicated_vertices[b])
                index_array_with_stride(normals, vertex_stride, vi) += n;
        } else {
            index_array_with_stride(normals, vertex_stride, i1) += n;
        }

        if (has_duplicates[i2]) {
            for (uint32_t vi : duplicated_vertices[c])
                index_array_with_stride(normals, vertex_stride, vi) += n;
        } else {
            index_array_with_stride(normals, vertex_stride, i2) += n;
        }
    }

    for (uint32_t i = 0; i < vertex_count; i++) {
        Vector3& n = index_array_with_stride(normals, vertex_stride, i);
        assert(n.length() > 1e-6f);
        n.normalize();
    }
}
