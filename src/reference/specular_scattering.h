#pragma once

#include "lib/color.h"

struct Scene_Object;
struct Thread_Context;

enum class Specular_Scattering_Type {
    none,
    specular_reflection,
    specular_transmission
};

struct Specular_Scattering {
    Specular_Scattering_Type type = Specular_Scattering_Type::none;
    ColorRGB scattering_coeff = Color_White;

    // Relative index of refraction, incident side relative to transmitted side.
    float etaI_over_etaT = 1.f;

    // Defines how BSDF-based scattering should be adjusted when material specifies both
    // BSDF function and delta scattering properties. In that case we split available
    // samples between finite and delta scattering events. This value is used to properly
    // weight finite/delta terms to get mathematically correct estimator. For materials
    // with only BSDF scattering this weight equals to unity.
    float finite_scattering_weight = 1.f;
};

Specular_Scattering get_specular_scattering_params(Thread_Context& thread_ctx, const Scene_Object* scene_objec);
bool trace_specular_bounces(Thread_Context& thread_ctx, int max_bounces, ColorRGB* specular_attenuation);
