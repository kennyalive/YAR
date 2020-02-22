#pragma once

#include "lib/color.h"
#include "lib/material.h"

struct Shading_Context;
struct Render_Context;
struct Vector3;

class BSDF {
public:
    virtual ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const = 0;
};

const BSDF* create_bsdf(const Render_Context& global_ctx, const Shading_Context& shading_ctx, Material_Handle material, void* bsdf_allocation, int bsdf_allocation_size);
