#pragma once

#include "lib/color.h"
#include "lib/vector.h"

struct Material_Data;

enum class Material_Type : uint32_t {
    none,
    diffuse
};

struct Material_Handle {
    Material_Type type;
    int index;
};
static_assert(sizeof(Material_Handle) == 8);
constexpr Material_Handle Null_Material = { Material_Type::none, -1 };

Material_Handle register_material(const Material_Data& material_data);
ColorRGB compute_bsdf(Material_Handle mtl, Vector3 wi, Vector3 wo);
