#include "std.h"
#include "lib/common.h"
#include "lambertian_brdf.h"

#include "../parameter_evaluation.h"

Lambertian_BRDF::Lambertian_BRDF(const Shading_Context& shading_ctx, const Lambertian_Material& material)
    : reflectance(evaluate_rgb_parameter(material.reflectance))
{}

ColorRGB Lambertian_BRDF::evaluate(const Vector3&, const Vector3&) const {
    return Pi_Inv * reflectance;
}