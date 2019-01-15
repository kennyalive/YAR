#include "obj_loader.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <unordered_map>

static inline bool operator==(const Mesh_Vertex& v, const Mesh_Vertex& v2) {
    return v.pos == v2.pos && v.normal == v2.normal && v.uv == v2.uv;
}

std::vector<Obj_Model> load_obj(const std::string& obj_file, const Mesh_Load_Params params)
{
    struct Vertex_Hasher {
        size_t operator()(const Mesh_Vertex& v) const {
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

    std::vector<Obj_Model> models(shapes.size());

    for (size_t i = 0; i < shapes.size(); i++) {
        tinyobj::shape_t& shape = shapes[i];
        Mesh_Data& mesh = models[i].mesh_data;

        std::unordered_map<Mesh_Vertex, uint32_t, Vertex_Hasher> unique_vertices;
        bool has_normals = true;

        for (const auto& index : shape.mesh.indices) {
            Mesh_Vertex vertex;

            vertex.pos = {
                attrib.vertices[3 * index.vertex_index + 0],
                attrib.vertices[3 * index.vertex_index + 1],
                attrib.vertices[3 * index.vertex_index + 2]
            };

            if (index.normal_index != -1) {
                vertex.normal = {
                    attrib.normals[3 * index.normal_index + 0],
                    attrib.normals[3 * index.normal_index + 1],
                    attrib.normals[3 * index.normal_index + 2],
                };
            } else {
                vertex.normal = Vector3_Zero;
                has_normals = false;
            }

            if (index.texcoord_index != -1) {
                vertex.uv = {
                    attrib.texcoords[2 * index.texcoord_index + 0],
                    1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
                };
            } else {
                vertex.uv = Vector2_Zero;
            }

            if (params.face_normals) {
                mesh.indices.push_back((uint32_t)mesh.vertices.size());
                mesh.vertices.push_back(vertex);
            } else {
                uint32_t index;
                if (unique_vertices.count(vertex) == 0) {
                    index = (uint32_t)mesh.vertices.size();
                    unique_vertices[vertex] = (uint32_t)mesh.vertices.size();
                    mesh.vertices.push_back(vertex);
                } else {
                    index = unique_vertices[vertex];
                }
                mesh.indices.push_back(index);
            }
        }

        if (params.face_normals) {
            for (int i = 0; i < mesh.indices.size(); i += 3) {
                uint32_t ia = mesh.indices[i + 0];
                uint32_t ib = mesh.indices[i + 1];
                uint32_t ic = mesh.indices[i + 2];
                Mesh_Vertex& va = mesh.vertices[ia];
                Mesh_Vertex& vb = mesh.vertices[ib];
                Mesh_Vertex& vc = mesh.vertices[ic];

                Vector3 n = cross(vb.pos - va.pos, vc.pos - va.pos).normalized();
                va.normal = n;
                vb.normal = n;
                vc.normal = n;
            }
        } else if (!has_normals) {
            std::vector<uint64_t> vertex_normal_groups;
            duplicate_vertices_due_to_crease_angle_threshold(mesh, vertex_normal_groups);
            ASSERT(vertex_normal_groups.size() == mesh.vertices.size());

            compute_normals(
                &mesh.vertices[0].pos,
                vertex_normal_groups.data(),
                (int)mesh.vertices.size(),
                (int)sizeof(Mesh_Vertex),
                mesh.indices.data(),
                (int)mesh.indices.size(),
                params.normal_average_mode,
                &mesh.vertices[0].normal
            );
        }

        if (!shape.mesh.material_ids.empty() && shape.mesh.material_ids[0] != -1) {
            for (int face_material_id : shape.mesh.material_ids)
                ASSERT(face_material_id == shape.mesh.material_ids[0]);

            const tinyobj::material_t& src = materials[shape.mesh.material_ids[0]];
            Obj_Material& mtl = models[i].material;
            mtl.k_diffuse = Vector3(src.diffuse);
            mtl.k_specular = Vector3(src.specular);
            models[i].has_material = true;
        } else {
            models[i].has_material = false;
        }
    }

    if (!params.transform.is_identity()) {
        for (Obj_Model& model : models) {
            for (Mesh_Vertex& v : model.mesh_data.vertices) {
                v.pos = transform_point(params.transform, v.pos);
                v.normal = transform_vector(params.transform, v.normal).normalized();
            }
        }
    }
    return models;
}
