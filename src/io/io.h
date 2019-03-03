#pragma once

#include "io/obj_loader.h"
#include "lib/color.h"
#include "lib/mesh.h"

#include <vector>

struct RGB_Point_Light_Data {
    Vector3     position;
    ColorRGB    intensity;
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
    std::vector<RGB_Point_Light_Data> rgb_point_lights;
    std::vector<Matrix3x4> view_points; // predefined camera positions
};

void write_exr_image(const char* file_name, const ColorRGB* pixels, int w, int h);
