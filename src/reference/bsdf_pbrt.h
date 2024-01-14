#pragma once

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
    ColorRGB sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const override;
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
    ColorRGB sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const override;
    float pdf(const Vector3& wo, const Vector3& wi) const override;

    const Pbrt3_Fourier_Material& data;
};
