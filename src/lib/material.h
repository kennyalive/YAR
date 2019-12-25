#pragma once

#include "color.h"

enum class Material_Type : uint32_t {
    none,
    lambertian 
};

struct Material_Handle {
    Material_Type type = Material_Type::none;
    int index = -1;
};

static_assert(sizeof(Material_Handle) == 8);
constexpr Material_Handle Null_Material = { Material_Type::none, -1 };

struct Lambertian_Material {
    ColorRGB albedo;
    int albedo_texture_index = -1;
};

struct Materials {
    std::vector<std::string> texture_names;
    std::vector<Lambertian_Material> lambertian;
};

