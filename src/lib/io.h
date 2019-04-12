#pragma once

#include "color.h"
#include "geometry.h"
#include "matrix.h"
#include "mesh.h"
#include "obj_loader.h"
#include "vector.h"

struct RGB_Point_Light_Data {
    Vector3 position;
    ColorRGB intensity;
};

struct RGB_Diffuse_Rectangular_Light_Data {
    Matrix3x4 light_to_world_transform;
    ColorRGB emitted_radiance;
    Vector2 size;
    int shadow_ray_count;
};

enum class Material_Format {
    obj_material
};

struct Material_Data {
    Material_Data() {}

    Material_Format material_format;
    union {
        Obj_Material obj_material;
    };
};

struct Scene_Data {
    std::string project_dir;
    std::vector<Mesh_Data> meshes;
    std::vector<Material_Data> materials; // per mesh material
    std::vector<Matrix3x4> view_points; // predefined camera positions

    // Lights
    std::vector<RGB_Point_Light_Data> rgb_point_lights;
    std::vector<RGB_Diffuse_Rectangular_Light_Data> rgb_diffuse_rectangular_lights;
};

// YAR file format
enum class Scene_Type {
    test_scene,
    pbrt_scene
};

struct YAR_Project {
    Scene_Type scene_type;
    std::string scene_path;
    Vector2i image_resolution;
    Bounds2i render_region;
    Matrix3x4 camera_to_world;
};

YAR_Project parse_project(const std::string& file_name);
bool save_project(const std::string& file_name, const YAR_Project& project);

Scene_Data load_scene(const YAR_Project& project);
void write_exr_image(const char* file_name, const ColorRGB* pixels, int w, int h);
