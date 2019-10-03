#pragma once

#include "color.h"
#include "scene.h"
#include "triangle_mesh.h"

struct YAR_Project;

struct Obj_Material {
    ColorRGB k_diffuse;
    ColorRGB k_specular;
};

struct Obj_Model {
    std::string name;
    Triangle_Mesh mesh;
    int material_index = -1;
};

struct Obj_Data {
    std::vector<Obj_Material> materials;
    std::vector<Obj_Model> models;
};

Obj_Data load_obj(const std::string& obj_file, const Triangle_Mesh_Load_Params params = Triangle_Mesh_Load_Params{}, const std::vector<std::string>& ignore_geometry_names = {});
Scene load_obj_project(const YAR_Project& project);

