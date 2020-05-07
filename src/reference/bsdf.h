#pragma once

#include "lib/material.h"

struct Render_Context;
struct Shading_Context;
struct Thread_Context;

class BSDF {
public:
    virtual ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const = 0;
};

const BSDF* create_bsdf(const Render_Context& global_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, Material_Handle material);

class Lambertian_BRDF : public BSDF {
public:
    Lambertian_BRDF(const Render_Context& global_ctx, const Shading_Context& shading_ctx, const Lambertian_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;

private:
    ColorRGB reflectance;
};
