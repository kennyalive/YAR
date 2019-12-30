#pragma once

#include "color.h"
#include "parameter.h"

enum class Material_Type : uint32_t {
    none,
    lambertian 
};

struct Material_Handle {
    Material_Type type = Material_Type::none;
    int index = -1;

    bool operator==(const Material_Handle& other) const {
        return type == other.type && index == other.index;
    }
    bool operator!=(const Material_Handle& other) const {
        return !(*this == other);
    }
};

static_assert(sizeof(Material_Handle) == 8);
constexpr Material_Handle Null_Material = { Material_Type::none, -1 };

struct Lambertian_Material {
    RGB_Parameter reflectance;
};

struct Materials {
    std::vector<std::string> texture_names;
    std::vector<Lambertian_Material> lambertian;
};
