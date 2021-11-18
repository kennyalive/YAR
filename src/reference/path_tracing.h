#pragma once

#include "lib/color.h"

struct Differential_Rays;
struct Ray;
struct Thread_Context;

ColorRGB estimate_path_contribution(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays& differential_rays);
