#pragma once

#include "color.h"

enum class Material_Type : uint32_t {
    none,
    lambertian 
};

struct Material_Handle {
    Material_Type type;
    int index;
};

static_assert(sizeof(Material_Handle) == 8);
constexpr Material_Handle Null_Material = { Material_Type::none, -1 };

struct Lambertian_Material {
    ColorRGB albedo;
};

struct Materials {
    std::vector<Lambertian_Material> lambertian;
};

