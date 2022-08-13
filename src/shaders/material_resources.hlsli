#ifndef MATERIAL_RESOURCES_HLSL
#define MATERIAL_RESOURCES_HLSL

#include "shared_main.hlsli"
#include "shared_material.hlsli"

static const int Material_Lambertian = 0;
static const int Material_None = -1;

[[vk::binding(LAMBERTIAN_MATERIAL_BINDING, MATERIAL_SET_INDEX)]]
RWStructuredBuffer<Lambertian_Material> lambertian_materials;

#endif
