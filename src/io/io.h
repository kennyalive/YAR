#pragma once

#include "io/obj_loader.h"
#include "lib/mesh.h"
#include "reference_cpu/spectrum.h"
#include <vector>

struct RGB_Point_Light_Data {
    Vector3 position;
    RGB intensity;
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
    std::vector<Mesh_Data> meshes;
    std::vector<Material_Data> materials; // per mesh material
    std::vector<RGB_Point_Light_Data> rgb_point_lights;
};

struct RGB;
void write_exr_image(const char* file_name, const RGB* pixels, int w, int h);
