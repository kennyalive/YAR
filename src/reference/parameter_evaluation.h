#pragma once

#include "lib/color.h"

struct RGB_Parameter;
struct Float_Parameter;
struct Render_Context;
struct Shading_Context;

ColorRGB evaluate_rgb_parameter(const Render_Context& global_ctx, const Shading_Context& shading_ctx, const RGB_Parameter& param);
float evaluate_float_parameter(const  Render_Context& global_ctx, const Shading_Context& shading_ctx, const Float_Parameter& param);