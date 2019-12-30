#include "std.h"
#include "lib/common.h"

#include "lambertian_brdf.h"

const BSDF* create_bsdf(const Shading_Context& shading_ctx, const Materials& materials, Material_Handle material, void* bsdf_allocation, int bsdf_allocation_size) {
    switch (material.type) {
        case Material_Type::lambertian:
        {
            const Lambertian_Material& params = materials.lambertian[material.index];
            ASSERT(sizeof(Lambertian_Material) <= bsdf_allocation_size);
            return new (bsdf_allocation) Lambertian_BRDF(shading_ctx, params);
        }
        default:
        {
            ASSERT(false);
            return nullptr;
        }
    }
}
