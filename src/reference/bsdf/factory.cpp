#include "std.h"
#include "lib/common.h"

#include "lambertian_brdf.h"
#include "../render_context.h"

const BSDF* create_bsdf(const Render_Context& global_ctx, const Shading_Context& shading_ctx, Material_Handle material, void* bsdf_allocation, int bsdf_allocation_size) {
    switch (material.type) {
        case Material_Type::lambertian:
        {
            const Lambertian_Material& params = global_ctx.materials.lambertian[material.index];
            ASSERT(sizeof(Lambertian_Material) <= bsdf_allocation_size);
            return new (bsdf_allocation) Lambertian_BRDF(global_ctx, shading_ctx, params);
        }
        default:
        {
            ASSERT(false);
            return nullptr;
        }
    }
}
