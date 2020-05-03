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

ColorRGB schlick_fresnel(const ColorRGB& R0, float cos_theta);

struct GGX_Distribution {
    static float D(const Vector3& wh, const Vector3& n, float alpha);
    static float G(const Vector3& wi, const Vector3& wo, const Vector3& n, float alpha);
};
