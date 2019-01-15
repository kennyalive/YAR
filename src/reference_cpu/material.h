#pragma once

#include "lib/vector.h"
#include <vector>

struct Material_Data;

enum class Material_Type : uint32_t {
    diffuse
};

struct Material_Handle {
    Material_Type type;
    uint32_t index;
};
static_assert(sizeof(Material_Handle) == 8);

Material_Handle register_material(const Material_Data& material_data);
Vector3 compute_bsdf(Material_Handle mtl, Vector3 wi, Vector3 wo);

