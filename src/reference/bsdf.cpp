#include "std.h"
#include "lib/common.h"
#include "bsdf.h"

#include "context.h"
#include "parameter_evaluation.h"

const BSDF* create_bsdf(const Render_Context& global_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, Material_Handle material) {
    switch (material.type) {
    case Material_Type::lambertian:
    {
        const Lambertian_Material& params = global_ctx.materials.lambertian[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Lambertian_BRDF>();
        return new (bsdf_allocation) Lambertian_BRDF(global_ctx, shading_ctx, params);
    }
    default:
    {
        ASSERT(false);
        return nullptr;
    }
    }
}


Lambertian_BRDF::Lambertian_BRDF(const Render_Context& global_ctx, const Shading_Context& shading_ctx, const Lambertian_Material& material)
    : reflectance(evaluate_rgb_parameter(global_ctx, shading_ctx, material.reflectance))
{}

ColorRGB Lambertian_BRDF::evaluate(const Vector3&, const Vector3&) const {
    return Pi_Inv * reflectance;
}
