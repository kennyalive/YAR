#pragma once

#include "lib/color.h"
#include "lib/triangle_mesh.h"

struct Obj_Material {
    ColorRGB k_diffuse;
    ColorRGB k_specular;
};

struct Obj_Model {
    std::string name;
    Triangle_Mesh mesh;
    bool has_material;
    Obj_Material material;
};

std::vector<Obj_Model> load_obj(const std::string& obj_file, const Triangle_Mesh_Load_Params params = Triangle_Mesh_Load_Params{});
