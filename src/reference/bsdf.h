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

    // types of scattering modeled by this bsdf
    bool reflection_scattering = false;
    bool transmission_scattering = false;

    Vector3 normal;
    Vector3 tangent;
    Vector3 bitangent;

protected:
    // Transformation of directions between local coordinate system defined
    // by the normal and two tangent vectors and world space coordinate system.
    Vector3 local_to_world(const Vector3& local_direction) const;
    Vector3 world_to_local(const Vector3& world_direction) const;

    Vector3 sample_microfacet_normal(Vector2 u, const Vector3& wo, float alpha) const;
};

struct Diffuse_BRDF : public BSDF {
    ColorRGB reflectance;

    Diffuse_BRDF(const Thread_Context& thread_ctx, const Diffuse_Material& material);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;
};

struct Diffuse_Transmission_BSDF : public BSDF {
    ColorRGB reflectance;
    ColorRGB transmittance;

    Diffuse_Transmission_BSDF(const Thread_Context& thread_ctx, const Diffuse_Transmission_Material& material);
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

struct Rough_Glass_BSDF : public BSDF {
    ColorRGB reflectance;
    ColorRGB transmittance;
    float alpha = 0.f;
    float eta_o = 0.f;
    float eta_i = 0.f;

    // TEMP: BSDF interface needs to add support the third random variable, that can be used to
    // select reflection layer. In some cases it's possible to re-purpose u[0] by remapping it,
    // but in some cases it's not possible (as in the rough glass bsdf).
    RNG* rng = nullptr;

    Rough_Glass_BSDF(const Thread_Context& thread_ctx, const Glass_Material& params);
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

// Because BSDFs in this renderer always represent finite bsdf functions,
// the pbrt uber BSDF also is not responsible for delta reflection/transmission
// scattering. It is handled by the delta scattering pipeline.
struct Pbrt3_Uber_BRDF : public BSDF {
    ColorRGB opacity;
    ColorRGB diffuse_reflectance;
    ColorRGB specular_reflectance;
    float alpha = 0.f;
    float index_of_refraction = 1.f;

    Pbrt3_Uber_BRDF(const Thread_Context& thread_ctx, const Pbrt3_Uber_Material& params);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;
};

struct Pbrt3_Plastic_BRDF : public Plastic_BRDF {
    Vector3 original_shading_normal;

    Pbrt3_Plastic_BRDF(const Thread_Context& thread_ctx, const Plastic_Material& params);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
};

struct Pbrt3_Fourier_BSDF : public BSDF {
    Pbrt3_Fourier_BSDF(const Thread_Context& thread_ctx, const Pbrt3_Fourier_Material& params);
    ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const override;
    ColorRGB sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;

    const Pbrt3_Fourier_Material& data;
};

const BSDF* create_bsdf(Thread_Context& thread_ctx, Material_Handle material);
