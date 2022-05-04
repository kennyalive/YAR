#pragma once

#include "lib/color.h"
#include "lib/ray.h"
#include "lib/vector.h"

struct Scene_Object;
struct Thread_Context;

struct Delta_Scattering {
    // When material has both BSDF and a delta layer then this value is how often we sample
    // the delta layer for path generation as opposed to bsdf sampling. For materials with
    // only BSDF scattering it is 0, and for pure delta materials it is 1. This field is always
    // initialized by 'check_for_delta_scattering_event' function, even when it returns false.
    float delta_layer_selection_probability = 0.f;

    // How delta surface changes radiance. This includes reflectance parameters, fresnel
    // effects, radiance scaling due to IoR changes.
    ColorRGB attenuation = Color_White;

    // New direction after scattering on delta surface.
    Vector3 delta_direction;

    // For delta reflection/transmission we have analytical formulas to 'scatter' incident 
    // differentital rays.
    bool has_differential_rays = false;
    Differential_Rays differential_rays;
};

// Returns true if delta scattering event is happening, otherwise false.
bool check_for_delta_scattering_event(Thread_Context& thread_ctx, const Scene_Object* scene_object,
    Delta_Scattering* delta_scattering);
