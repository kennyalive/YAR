#pragma once

#include "color.h"
#include "triangle_mesh.h"

struct Obj_Material {
    ColorRGB k_diffuse;
    ColorRGB k_specular;
    String diffuse_texture;
};

struct Obj_Mesh {
    String name;
    Triangle_Mesh mesh;
    int material_index = -1;
};

struct Obj_Data {
    std::vector<Obj_Material> materials;
    std::vector<Obj_Mesh> meshes;
};

Obj_Data load_obj(
    const char* obj_file_path,
    const Triangle_Mesh_Load_Params& params = Triangle_Mesh_Load_Params{},
    const std::vector<String>* ignore_geometry_names = nullptr
);
