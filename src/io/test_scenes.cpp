#include "lib/common.h"
#include "test_scenes.h"
#include "io/obj_loader.h"
#include "reference/colorimetry.h"
#include "reference/spectrum.h"

static const Matrix3x4 from_obj_to_world {
    1, 0,  0, 0,
    0, 0, -1, 0,
    0, 1,  0, 0
};

ColorRGB convert_flux_to_constant_spectrum_to_rgb_intensity(float luminous_flux) {
    float radiant_flux_per_wavelength = luminous_flux / (683.f * CIE_Y_integral); // [W/m]
    // Uniform spectrum that produces luminous_flux
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(radiant_flux_per_wavelength);
    Vector3 xyz = s.emission_spectrum_to_XYZ();
    // Uniform spectrum does not produce white RGB (for sRGB). It's a bit reddish.
    return ColorRGBFromXYZ(xyz);
}

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
            material.obj_material.k_diffuse = Color_White;
        }
        scene.materials.push_back(material);
    }
    return scene;
}

Scene_Data load_bunny_scene() {
    RGB_Point_Light_Data light;
    light.position = Vector3(2, -2, 1.5);
    light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(1600 /*Lm*/);

    Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(from_obj_to_world, 1.f);
    std::vector<Obj_Model> obj_models = load_obj("bunny/bunny.obj", mesh_load_params);

    Scene_Data scene = convert_obj_models(obj_models);
    scene.project_dir = "bunny";
    scene.rgb_point_lights.push_back(light);

    Matrix3x4 view_point {
        0.942210f, -0.318238f, -0.104785f, 0.466048f,
        0.335043f, 0.894951f, 0.294679f, -2.158572f,
        0.000000f, -0.312751f, 0.949842f, 1.369773f,
    };
    scene.view_points.push_back(view_point);
    return scene;
}

Scene_Data load_conference_scene() {
    RGB_Point_Light_Data light;
    light.position = Vector3(2, 0, 1.5);
    light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(1600 /*Lm*/);

    RGB_Point_Light_Data light2;
    light2.position = Vector3(-1, 1, 1.0);
    light2.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(1600 /*Lm*/);

    Mesh_Load_Params mesh_load_params;
    mesh_load_params.crease_angle = radians(60);
    mesh_load_params.transform = uniform_scale(from_obj_to_world, 0.003f);
    std::vector<Obj_Model> obj_models = load_obj("conference/conference.obj", mesh_load_params);

    Scene_Data scene = convert_obj_models(obj_models);
    scene.project_dir = "conference";
    scene.rgb_point_lights.push_back(light);
    scene.rgb_point_lights.push_back(light2);

    Matrix3x4 view_point{
        -0.786632f, 0.589048f, 0.185115f, -0.329195f,
        -0.617444f, -0.750455f, -0.235839f, 2.223660f,
        0.000000f, -0.299808f, 0.954012f, 1.494759f
    };
    scene.view_points.push_back(view_point);
    return scene;
}

Scene_Data load_buddha_scene() {
    RGB_Point_Light_Data light;
    light.position = Vector3(2, 2, 1.5);
    light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(1600 /*Lm*/);

    Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(from_obj_to_world, 1.f);
    std::vector<Obj_Model> obj_models = load_obj("buddha/buddha.obj", mesh_load_params);

    Scene_Data scene = convert_obj_models(obj_models);
    scene.project_dir = "buddha";
    scene.rgb_point_lights.push_back(light);

    Matrix3x4 view_point {
        -0.990574f, 0.136961f, 0.003766f, -0.147305f,
        -0.137013f, -0.990206f, -0.027226f, 1.083111f,
        0.000000f, -0.027486f, 0.999627f, 0.058400f,
    };
    scene.view_points.push_back(view_point);
    return scene;
}

Scene_Data load_hairball_scene() {
    RGB_Point_Light_Data light;
    light.position = Vector3(2, 2, 1.5);
    light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(1600 /*Lm*/);

    Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(from_obj_to_world, 1.f);
    mesh_load_params.invert_winding_order = true;
    std::vector<Obj_Model> obj_models = load_obj("hairball/hairball.obj", mesh_load_params);

    Scene_Data scene = convert_obj_models(obj_models);
    scene.project_dir = "hairball";
    scene.rgb_point_lights.push_back(light);

    Matrix3x4 view_point {
        -0.981547f, -0.190761f, -0.013507f, 1.663855f,
        0.191238f, -0.979099f, -0.069324f, 9.265212f,
        0.000000f, -0.070627f, 0.997506f, 0.618077f
    };
    scene.view_points.push_back(view_point);
    return scene;
}

Scene_Data load_mori_knob() {
    Vector2 light_size { 1, 1 }; // 1 m^2 light

    // Convert provided luminous flux to radiant flux assuming constant spectrum.
    float luminous_flux = 3000; // [Lm]
    float radiant_flux_per_wavelength = luminous_flux / (683.f * CIE_Y_integral); // [W/m]
    float radiant_exitance_per_wavelength = Pi * radiant_flux_per_wavelength; // [M/m]
    
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(radiant_exitance_per_wavelength);
    Vector3 xyz = s.emission_spectrum_to_XYZ();

    Mesh_Load_Params mesh_load_params;
    mesh_load_params.transform = uniform_scale(from_obj_to_world, 1.f);
    std::vector<Obj_Model> obj_models = load_obj("mori_knob/testObj.obj", mesh_load_params);

    Scene_Data scene = convert_obj_models(obj_models);
    scene.project_dir = "mori_knob";

    RGB_Diffuse_Rectangular_Light_Data rect_light;
    rect_light.light_to_world_transform = Matrix3x4{
        -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 1
    };
    rect_light.emitted_radiance = ColorRGBFromXYZ(xyz);
    rect_light.size = light_size;
    rect_light.shadow_ray_count = 1;
    scene.rgb_diffuse_rectangular_lights.push_back(rect_light);

    Matrix3x4 view_point{
        -0.954639f, 0.265867f, 0.134153f, -0.833258f,
        -0.297793f, -0.852289f, -0.430056f, 1.268962f,
        0.000000f, -0.450491f, 0.892788f, 0.055605f,

    };
    scene.view_points.push_back(view_point);
    return scene;
}
