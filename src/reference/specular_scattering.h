#pragma once

#include "lib/color.h"
#include "lib/vector.h"

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

    // For materials with both finite and delta layers this flag signals that the
    // next path segment should be traced along delta_direction.
    bool sample_delta_direction = false;
    Vector3 delta_direction;
    float finite_scattering_weight = 1.f;
};

Specular_Scattering get_specular_scattering_params(Thread_Context& thread_ctx, const Scene_Object* scene_objec);
bool trace_specular_bounces(Thread_Context& thread_ctx, int max_bounces, ColorRGB* specular_attenuation);
