#pragma once

#include "lib/color.h"
#include "lib/material.h"

struct Auxilary_Rays;
struct Scene_Context;
struct Thread_Context;

enum class Specular_Scattering_Type {
    none,
    specular_reflection,
    specular_transmission
};

struct Specular_Scattering {
    Specular_Scattering_Type type = Specular_Scattering_Type::none;
    ColorRGB scattering_coeff = Color_White;
    float etaI_over_etaT = 1.f; // relative index of refracton, incident side relative to transmitted side
};

Specular_Scattering get_specular_scattering_params(Thread_Context& thread_ctx, Material_Handle material_handle);

bool trace_specular_bounces(Thread_Context& thread_ctx, const Auxilary_Rays* incident_auxilary_rays,
    int max_bounces, int* bounce_counter /*in/out*/, ColorRGB* specular_bounces_contribution);
