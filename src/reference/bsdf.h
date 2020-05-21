#pragma once

#include "lib/material.h"

struct Render_Context;
struct Shading_Context;
struct Thread_Context;

struct BSDF {
    virtual ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const = 0;
};

const BSDF* create_bsdf(const Render_Context& global_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, Material_Handle material);

struct Lambertian_BRDF : public BSDF {
    ColorRGB reflectance;

    Lambertian_BRDF(const Render_Context& global_ctx, const Shading_Context& shading_ctx, const Lambertian_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
};

struct Metal_BRDF : public BSDF {
    float roughness = 0.f;
    float eta_i; // ROI of adjacent dielectric
    ColorRGB eta_t;
    ColorRGB k_t;

    const Render_Context* scene_ctx;
    const Shading_Context* shading_ctx;

    Metal_BRDF(const Render_Context& global_ctx, const Shading_Context& shading_ctx, const Metal_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
};
