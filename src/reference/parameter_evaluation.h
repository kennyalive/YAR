#pragma once

#include "lib/color.h"

struct RGB_Parameter;
struct Float_Parameter;
struct Thread_Context;

ColorRGB evaluate_rgb_parameter(const Thread_Context& thread_ctx, const RGB_Parameter& param);
float evaluate_float_parameter(const Thread_Context& thread_ctx, const Float_Parameter& param);
