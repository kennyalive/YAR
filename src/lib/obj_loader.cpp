#include "std.h"
#include "common.h"

#include "obj_loader.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny/tiny_obj_loader.h>

#include <unordered_map>

namespace {
struct Mesh_Vertex {
    Vector3 pos;
    Vector3 normal;
    Vector2 uv;

    bool operator==(const Mesh_Vertex& other) const {
        return pos == other.pos && normal == other.normal && uv == other.uv;
    }
};

struct Mesh_Vertex_Hasher {
    size_t operator()(const Mesh_Vertex& v) const {
        size_t hash = 0;
        hash_combine(hash, v.pos);
        hash_combine(hash, v.normal);
        hash_combine(hash, v.uv);
        return hash;
    }
};
}

static void convert_tinyobj_shape_to_meshes(const tinyobj::shape_t& shape, tinyobj::attrib_t attrib,
    const Triangle_Mesh_Load_Params params, std::vector<Obj_Mesh>& obj_meshes)
{
    int current_material_id = shape.mesh.material_ids.empty() ? -1 : shape.mesh.material_ids[0];
    std::unordered_map<Mesh_Vertex, uint32_t, Mesh_Vertex_Hasher> unique_vertices;
    bool has_normals = true;

    obj_meshes.push_back(Obj_Mesh{});
    obj_meshes.back().name = shape.name;
    obj_meshes.back().material_index = current_material_id;
    Triangle_Mesh* mesh = &obj_meshes.back().mesh;

    for (int i = 0; i < (int)shape.mesh.indices.size(); i++) {
        const tinyobj::index_t& index = shape.mesh.indices[i];

        // Start new mesh if new material is found.
        if (!shape.mesh.material_ids.empty() && i > 0 && i%3 == 0) {
            int face_index = i/3;
            int material_id = shape.mesh.material_ids[face_index];
            if (material_id != current_material_id) {
                if (!has_normals || params.force_normal_calculation)
                    calculate_normals(params.normal_calculation_params, *mesh);

                obj_meshes.push_back(Obj_Mesh{});
                obj_meshes.back().name = shape.name;
                obj_meshes.back().material_index = material_id;
                mesh = &obj_meshes.back().mesh;
                current_material_id = material_id;
                unique_vertices.clear();
                has_normals = true;
            }
        }

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

        uint32_t vertex_index;
        if (unique_vertices.count(vertex) == 0) {
            vertex_index = (uint32_t)mesh->vertices.size();
            unique_vertices[vertex] = (uint32_t)mesh->vertices.size();
            mesh->vertices.push_back(vertex.pos);
            mesh->normals.push_back(vertex.normal);
            mesh->uvs.push_back(vertex.uv);
        } else {
            vertex_index = unique_vertices[vertex];
        }
        mesh->indices.push_back(vertex_index);
    }

    if (!has_normals || params.force_normal_calculation)
        calculate_normals(params.normal_calculation_params, *mesh);
}

Obj_Data load_obj(
    const std::string& obj_file_path,
    const Triangle_Mesh_Load_Params& params,
    const std::vector<std::string>* ignore_geometry_names)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    const std::string mtl_dir = fs::path(obj_file_path).parent_path().string();

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, obj_file_path.c_str(), mtl_dir.c_str()))
        error("failed to load obj model: " + obj_file_path);

    Obj_Data obj_data;

    obj_data.materials.resize(materials.size());
    for (auto [i, tinyobj_mtl] : enumerate(materials)) {
        Obj_Material& mtl = obj_data.materials[i];
        mtl.k_diffuse = ColorRGB{tinyobj_mtl.diffuse};
        mtl.k_specular = ColorRGB{tinyobj_mtl.specular};
        if (!tinyobj_mtl.diffuse_texname.empty())
            mtl.diffuse_texture = tinyobj_mtl.diffuse_texname;
    }

    for (const tinyobj::shape_t& shape : shapes) {
        if (ignore_geometry_names) {
            if (std::find(ignore_geometry_names->begin(), ignore_geometry_names->end(), shape.name) != ignore_geometry_names->end()) {
                continue;
            }
        }
        convert_tinyobj_shape_to_meshes(shape, attrib, params, obj_data.meshes);
    }

    if (!params.transform.is_identity()) {
        for (Obj_Mesh& obj_mesh : obj_data.meshes) {
            for (Vector3& p : obj_mesh.mesh.vertices)
                p = transform_point(params.transform, p);
            for (Vector3& n : obj_mesh.mesh.normals)
                n = transform_vector(params.transform, n).normalized();
        }
    }
    if (params.invert_winding_order) {
        for (Obj_Mesh& obj_mesh: obj_data.meshes) {
            for (int i = 0; i < (int)obj_mesh.mesh.indices.size(); i += 3)
                std::swap(obj_mesh.mesh.indices[i], obj_mesh.mesh.indices[i+1]);
        }
    }
    return obj_data;
}
