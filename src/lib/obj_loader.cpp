#include "std.h"
#include "common.h"
#include "obj_loader.h"

#include "colorimetry.h"
#include "project.h"
#include "spectrum.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

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
} // namespace

Obj_Data load_obj(const std::string& obj_file, const Triangle_Mesh_Load_Params params, const std::vector<std::string>& ignore_geometry_names)
{
    // Load obj file.
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    const std::string obj_path = get_resource_path(obj_file).c_str();
    const std::string mtl_dir = get_directory(obj_path);

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, obj_path.c_str(), mtl_dir.c_str()))
        error("failed to load obj model: " + obj_file);

    Obj_Data obj_data;

    // Get materials.
    obj_data.materials.resize(materials.size());
    for (auto [i, tinyobj_mtl] : enumerate(materials)) {
        Obj_Material& mtl = obj_data.materials[i];
        mtl.k_diffuse = ColorRGB{tinyobj_mtl.diffuse};
        mtl.k_specular = ColorRGB{tinyobj_mtl.specular};
    }

    // Get objects.
    obj_data.models.reserve(shapes.size());
    auto& models = obj_data.models;

    for (size_t i = 0; i < shapes.size(); i++) {
        tinyobj::shape_t& shape = shapes[i];

        if (std::find(ignore_geometry_names.begin(), ignore_geometry_names.end(), shape.name) != ignore_geometry_names.end()) {
            continue;
        }

        obj_data.models.push_back(Obj_Model{});
        Obj_Model& model = obj_data.models.back();
        model.name = shape.name;
        Triangle_Mesh& mesh = model.mesh;

        std::unordered_map<Mesh_Vertex, uint32_t, Mesh_Vertex_Hasher> unique_vertices;
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
                mesh.vertices.push_back(vertex.pos);
                mesh.normals.push_back(vertex.normal);
                mesh.uvs.push_back(vertex.uv);
            } else {
                uint32_t index;
                if (unique_vertices.count(vertex) == 0) {
                    index = (uint32_t)mesh.vertices.size();
                    unique_vertices[vertex] = (uint32_t)mesh.vertices.size();
                    mesh.vertices.push_back(vertex.pos);
                    mesh.normals.push_back(vertex.normal);
                    mesh.uvs.push_back(vertex.uv);
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
                const Vector3& va = mesh.vertices[ia];
                const Vector3& vb = mesh.vertices[ib];
                const Vector3& vc = mesh.vertices[ic];

                Vector3 n = cross(vb - va, vc - va).normalized();
                mesh.normals[ia] = n;
                mesh.normals[ib] = n;
                mesh.normals[ic] = n;
            }
        } else if (!has_normals) {
            compute_normals(mesh, params.normal_average_mode, params.crease_angle);
        }

        if (!shape.mesh.material_ids.empty() && shape.mesh.material_ids[0] != -1) {
            // Check that all faces have the same material. Multi-material meshes are not supported.
            /*
            for (int face_material_id : shape.mesh.material_ids) {
                ASSERT(face_material_id == shape.mesh.material_ids[0]);
            }
            */
            model.material_index = shape.mesh.material_ids[0];
        }
    }

    if (!params.transform.is_identity()) {
        for (Obj_Model& model : models) {
            for (Vector3& p : model.mesh.vertices)
                p = transform_point(params.transform, p);
            for (Vector3& n : model.mesh.normals)
                n = transform_vector(params.transform, n).normalized();
        }
    }

    if (params.invert_winding_order) {
        for (Obj_Model& model : models) {
            for (int i = 0; i < (int)model.mesh.indices.size(); i += 3)
                std::swap(model.mesh.indices[i], model.mesh.indices[i+1]);
        }
    }
    return obj_data;
}

static const Matrix3x4 from_obj_to_world {
    1, 0,  0, 0,
    0, 0, -1, 0,
    0, 1,  0, 0
};

Scene load_obj_project(const YAR_Project& project) {
    Triangle_Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(from_obj_to_world, project.world_scale);
    mesh_load_params.crease_angle = project.mesh_crease_angle;
    mesh_load_params.invert_winding_order = project.mesh_invert_winding_order;
    Obj_Data obj_data = load_obj(project.scene_path, mesh_load_params, project.ignore_geometry_names);

    Scene scene;

    scene.materials.lambertian.resize(obj_data.materials.size());
    for (auto [i, obj_material] : enumerate(obj_data.materials)) {
        scene.materials.lambertian[i].albedo = obj_material.k_diffuse;
    }

    scene.geometries.triangle_meshes.resize(obj_data.models.size());
    // We can have more elements in case of instancing.
    scene.render_objects.reserve(obj_data.models.size()); 

    std::map<std::string, std::vector<YAR_Instance>> instance_infos;
    for (const YAR_Instance& instance : project.instances)
        instance_infos[instance.geometry_name].push_back(instance);

    bool add_default_material = false;
    for (int i = 0; i < (int)obj_data.models.size(); i++) {
        scene.geometries.triangle_meshes[i] = obj_data.models[i].mesh;

        Material_Handle material;
        //if (obj_data.models[i].material_index == -1) {
        material = { Material_Type::lambertian, (int)scene.materials.lambertian.size() };
        add_default_material = true;
        //}
        //else {
        //    material = { Material_Type::lambertian, obj_data.models[i].material_index };
        //}

        auto instances_it = instance_infos.find(obj_data.models[i].name); 
        if (instances_it != instance_infos.end()) {
            for (const YAR_Instance& instance : instances_it->second) {
                scene.render_objects.push_back(Render_Object{});
                Render_Object& render_object = scene.render_objects.back();
                render_object.geometry = { Geometry_Type::triangle_mesh, i };
                render_object.material = material;
                render_object.object_to_world_transform = instance.transform;
                render_object.world_to_object_transform = get_inverted_transform(instance.transform);
            }
        }
        else {
            scene.render_objects.push_back(Render_Object{});
            Render_Object& render_object = scene.render_objects.back();
            render_object.geometry = { Geometry_Type::triangle_mesh, i };
            render_object.material = material;
            render_object.world_to_object_transform = Matrix3x4::identity;
            render_object.object_to_world_transform = Matrix3x4::identity;

        }
    }
    if (add_default_material) {
        scene.materials.lambertian.push_back({Color_White});
    }

    scene.lights = project.lights;
    scene.fovy = 45.f;
    return scene;
}
