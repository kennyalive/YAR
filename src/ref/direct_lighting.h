#pragma once

#include "lib/color.h"
#include "lib/vector.h"

struct Differential_Rays;
struct Ray;
struct Thread_Context;

ColorRGB get_emitted_radiance(Thread_Context& thread_ctx);

ColorRGB estimate_direct_lighting(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays& differential_rays);

ColorRGB estimate_direct_lighting_from_single_sample(const Thread_Context& thread_ctx,
    float u_light_selector, Vector2 u_light, Vector2 u_bsdf, float u_scattering_type);
