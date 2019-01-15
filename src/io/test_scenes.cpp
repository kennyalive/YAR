#include "test_scenes.h"
#include "io/obj_loader.h"
#include "reference_cpu/colorimetry.h"
#include "reference_cpu/spectrum.h"

static Scene_Data convert_obj_models(const std::vector<Obj_Model>& obj_models) {
    Scene_Data scene;

    scene.meshes.reserve(obj_models.size());
    scene.materials.reserve(obj_models.size());

    for (const Obj_Model& model : obj_models) {
        scene.meshes.push_back(model.mesh_data);

        Material_Data material{};
        material.material_format = Material_Format::obj_material;

        if (model.has_material)
            material.obj_material = model.material;
        else {
            material.obj_material.k_diffuse = Vector3(1);
        }
        scene.materials.push_back(material);
    }
    return scene;
}

Scene_Data load_conference_scene() {
    Matrix3x4 to_world_axis_orientation {{
        {1, 0,  0, 0},
        {0, 0, -1, 0},
        {0, 1,  0, 0}
    }};

    // Uniform spectrum that produces luminous flux of 1600Lm.
    float P = 1600; // Lm
    float C = P / (683.f * CIE_Y_integral); // [W/m]
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(C);
    XYZ xyz = s.emission_spectrum_to_XYZ();

    RGB_Point_Light_Data light;
    light.position = Vector3(2, 0, 1.5);
    light.intensity = RGB(xyz);

    RGB_Point_Light_Data light2;
    light2.position = Vector3(-1, -1, 1.0);
    light2.intensity = RGB(xyz);

    Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(to_world_axis_orientation, 0.003f);

    std::vector<Obj_Model> obj_models = load_obj("conference/conference.obj", mesh_load_params);

    Scene_Data scene = convert_obj_models(obj_models);
    scene.rgb_point_lights.push_back(light);
    scene.rgb_point_lights.push_back(light2);
    return scene;
}

Scene_Data load_bunny_scene() {
    Matrix3x4 to_world_axis_orientation {{
        {1, 0,  0, 0},
        {0, 0, -1, 0},
        {0, 1,  0, 0}
    }};

    // Uniform spectrum that produces luminous flux of 1600Lm.
    float P = 1600; // Lm
    float C = P / (683.f * CIE_Y_integral); // [W/m]
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(C);
    XYZ xyz = s.emission_spectrum_to_XYZ();

    RGB_Point_Light_Data light;
    light.position = Vector3(2, 2, 1.5);
    light.intensity = RGB(xyz);

    Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(to_world_axis_orientation, 1.f);

    std::vector<Obj_Model> obj_models = load_obj("bunny/bunny.obj", mesh_load_params);

    Scene_Data scene = convert_obj_models(obj_models);
    scene.rgb_point_lights.push_back(light);
    return scene;
}
