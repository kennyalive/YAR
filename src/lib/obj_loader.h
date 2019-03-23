#pragma once

#include "lib/color.h"
#include "lib/mesh.h"

struct Obj_Material {
    ColorRGB k_diffuse;
    ColorRGB k_specular;
};

struct Obj_Model {
    Mesh_Data mesh_data;
    bool has_material;
    Obj_Material material;
};

std::vector<Obj_Model> load_obj(const std::string& obj_file, const Mesh_Load_Params params = Mesh_Load_Params{});
