#pragma once

#include "lib/color.h"

struct Differential_Rays;
struct Ray;
struct Thread_Context;

ColorRGB trace_path(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays& differential_rays);
