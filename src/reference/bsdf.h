#pragma once

#include "lib/material.h"
#include "lib/vector.h"

struct Shading_Context;
struct Thread_Context;

struct BSDF {
    virtual ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const = 0;
    virtual ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const = 0;
    virtual float pdf(const Vector3& wo, const Vector3& wi) const = 0;

    BSDF(const Shading_Context& shading_ctx);

    const Shading_Context* shading_ctx;
    const Vector3 n; // shading normal (extracted from shading_ctx)

    // types of scattering modeled by this bsdf
    bool reflection_scattering = false;
    bool transmission_scattering = false;
};

struct Lambertian_BRDF : public BSDF {
    ColorRGB reflectance;

    Lambertian_BRDF(const Thread_Context& thread_ctx, const Lambertian_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;
};

struct Metal_BRDF : public BSDF {
    float alpha = 0.f;
    float eta_i = 0.f; // ROI of adjacent dielectric
    ColorRGB eta_t;
    ColorRGB k_t;

    Metal_BRDF(const Thread_Context& thread_ctx, const Metal_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;
};

struct Plastic_BRDF : public BSDF {
    float alpha = 0.f;
    float r0 = 0.f;
    ColorRGB diffuse_reflectance;

    Plastic_BRDF(const Thread_Context& thread_ctx, const Plastic_Material& params);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;
};

struct Ashikhmin_Shirley_Phong_BRDF : public BSDF {
    float alpha = 0.f; // GGX alpha parameter
    ColorRGB r0; // reflectance of the glossy layer at normal incident angle
    ColorRGB diffuse_reflectance; // reflectance of the diffuse layer

    Ashikhmin_Shirley_Phong_BRDF(const Thread_Context& thread_ctx, const Coated_Diffuse_Material& params);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;
};

const BSDF* create_bsdf(Thread_Context& thread_ctx, Material_Handle material);
