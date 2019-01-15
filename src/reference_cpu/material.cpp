#include "material.h"
#include "io/io.h"

namespace {
struct Diffuse_Material {
    Vector3 albedo;
};

struct Materials {
    std::vector<Diffuse_Material> diffuse;
};
}

static Materials materials;

Material_Handle register_material(const Material_Data& material_data) {
    if (material_data.material_format == Material_Format::obj_material) {
        const Obj_Material& obj_material = material_data.obj_material;
        materials.diffuse.push_back(Diffuse_Material{ obj_material.k_diffuse });
        return Material_Handle{ Material_Type::diffuse, uint32_t(materials.diffuse.size() - 1) };
    }

    ASSERT(false);
    return Material_Handle{};
}

Vector3 compute_bsdf(Material_Handle mtl, Vector3 wi, Vector3 wo) {
    switch (mtl.type) {
        case Material_Type::diffuse:
            ASSERT(mtl.index < materials.diffuse.size());
            return materials.diffuse[mtl.index].albedo * Pi_Inv;
        default:
            ASSERT(false);
            return Vector3_Zero;
    }
}
