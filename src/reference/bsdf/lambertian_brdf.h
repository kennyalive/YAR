#pragma once

#include "../scattering.h"
#include "lib/material.h"

struct Shading_Context;

class Lambertian_BRDF : public BSDF {
public:
    Lambertian_BRDF(const Shading_Context& shading_ctx, const Lambertian_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;

private:
    ColorRGB reflectance;
};
