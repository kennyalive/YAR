#include "std.h"
#include "common.h"

#include "colorimetry.h"
#include "obj_loader.h"
#include "scene.h"
#include "spectrum.h"
#include "yar_project.h"

Scene load_obj_scene(const YAR_Project& project) {
    Triangle_Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(Matrix3x4::identity, project.world_scale);
    mesh_load_params.normal_calculation_params.use_crease_angle = project.mesh_use_crease_angle;
    mesh_load_params.normal_calculation_params.crease_angle = project.mesh_crease_angle;
    mesh_load_params.invert_winding_order = project.mesh_invert_winding_order;
    Obj_Data obj_data = load_obj(project.scene_path.string(), mesh_load_params, &project.ignore_geometry_names);

    Scene scene;

    scene.materials.lambertian.resize(obj_data.materials.size());
    for (auto [i, obj_material] : enumerate(obj_data.materials)) {
        if (obj_material.diffuse_texture.empty()) {
            scene.materials.lambertian[i].reflectance.is_constant = true;
            scene.materials.lambertian[i].reflectance.constant_value = obj_material.k_diffuse;
        }
        else {
            scene.materials.lambertian[i].reflectance.is_constant = false;
            scene.texture_names.push_back(obj_material.diffuse_texture);
            scene.materials.lambertian[i].reflectance.texture_index = (int)scene.texture_names.size()-1;
        }
    }

    scene.geometries.triangle_meshes.resize(obj_data.meshes.size());
    // We can have more elements in case of instancing.
    scene.objects.reserve(obj_data.meshes.size()); 

    std::map<std::string, std::vector<YAR_Instance>> instance_infos;
    for (const YAR_Instance& instance : project.instances)
        instance_infos[instance.geometry_name].push_back(instance);

    bool add_default_material = false;
    for (int i = 0; i < (int)obj_data.meshes.size(); i++) {
        scene.geometries.triangle_meshes[i] = obj_data.meshes[i].mesh;

        Material_Handle material;
        if (obj_data.meshes[i].material_index == -1) {
            material = { Material_Type::lambertian, (int)scene.materials.lambertian.size() };
            add_default_material = true;
        }
        else {
            material = { Material_Type::lambertian, obj_data.meshes[i].material_index };
        }

        auto instances_it = instance_infos.find(obj_data.meshes[i].name); 
        if (instances_it != instance_infos.end()) {
            for (const YAR_Instance& instance : instances_it->second) {
                scene.objects.push_back(Scene_Object{});
                Scene_Object& scene_object = scene.objects.back();
                scene_object.geometry = { Geometry_Type::triangle_mesh, i };
                scene_object.material = material;
                scene_object.object_to_world_transform = instance.transform;
                scene_object.world_to_object_transform = get_inverted_transform(instance.transform);
            }
        }
        else {
            scene.objects.push_back(Scene_Object{});
            Scene_Object& scene_object = scene.objects.back();
            scene_object.geometry = { Geometry_Type::triangle_mesh, i };
            scene_object.material = material;
            scene_object.world_to_object_transform = Matrix3x4::identity;
            scene_object.object_to_world_transform = Matrix3x4::identity;

        }
    }
    if (add_default_material) {
        scene.materials.lambertian.push_back({true, Color_White, -1});
    }

    scene.lights = project.lights;
    return scene;
}
