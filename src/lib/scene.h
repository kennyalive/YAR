#pragma once

#include "geometry.h"
#include "light.h"
#include "material.h"
#include "matrix.h"
#include "raytracer_config.h"
#include "scene_object.h"

enum class Scene_Type {
    pbrt,
    obj,
};

struct Texture_Descriptor {
    std::string file_name;
    bool decode_srgb = false;
    float scale = 1.f;

    bool is_constant_texture = false;
    ColorRGB constant_value = Color_Black;

    bool operator==(const Texture_Descriptor&) const = default;
};

struct Scene {
    Scene_Type type;
    std::string path;

    // Optional filename of the output image.
    std::string output_filename;

    //
    // Renderer configuration
    //
    Vector2i film_resolution;
    Bounds2i render_region;
    float camera_fov_y = 0.f;
    bool  z_is_up = false;
    bool mesh_disable_backfacing_culling = false;
    bool front_face_has_clockwise_winding = false;
    Raytracer_Config raytracer_config;

    std::vector<Texture_Descriptor> texture_descriptors;
    std::vector<Parameter> material_parameters;

    // Predefined camera positions.
    std::vector<Matrix3x4> view_points;

    // Cache of tesselated spheres with different radius
    std::unordered_map<float, Geometry_Handle> radius_to_sphere_geometry;

    //
    // Description of the virtual environment.
    //
    Geometries geometries;
    Materials materials;
    Lights lights;
    std::vector<Scene_Object> objects;

    std::string get_resource_absolute_path(const std::string& resource_relative_path) const
    {
        return (fs::path(path).parent_path() / resource_relative_path).string();
    }
};
