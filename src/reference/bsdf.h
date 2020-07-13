#pragma once

#include "lib/material.h"
#include "lib/vector.h"

struct Scene_Context;
struct Shading_Context;
struct Thread_Context;

struct BSDF {
    virtual ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const = 0;
    virtual ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const = 0;
    virtual float pdf(const Vector3& wo, const Vector3& wi) const = 0;

    BSDF(const Shading_Context& shading_ctx);

    const Shading_Context* shading_ctx;
    Vector3 N; // shading normal (extracted from shading_ctx)

    // types of scattering modeled by this bsdf
    bool reflection_scattering = false;
    bool transmission_scattering = false;
};

const BSDF* create_bsdf(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, Material_Handle material);

struct Lambertian_BRDF : public BSDF {
    ColorRGB reflectance;

    Lambertian_BRDF(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Lambertian_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;
};

struct Metal_BRDF : public BSDF {
    float roughness = 0.f;
    float eta_i; // ROI of adjacent dielectric
    ColorRGB eta_t;
    ColorRGB k_t;

    const Scene_Context* scene_ctx;
    const Shading_Context* shading_ctx;

    Metal_BRDF(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Metal_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override { ASSERT(false); return ColorRGB{}; }
    float pdf(const Vector3& wo, const Vector3& wi) const override { ASSERT(false); return 0.f; }
};

struct Plastic_BRDF : public BSDF {
    float roughness = 0.f;
    float r0;
    ColorRGB diffuse_reflectance;

    const Scene_Context* scene_ctx;
    const Shading_Context* shading_ctx;

    Plastic_BRDF(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Plastic_Material& params);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override { ASSERT(false); return ColorRGB{}; }
    float pdf(const Vector3& wo, const Vector3& wi) const override { ASSERT(false); return 0.f; }
};
