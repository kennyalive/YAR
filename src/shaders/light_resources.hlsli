#ifndef LIGHT_RESOURCES_HLSL
#define LIGHT_RESOURCES_HLSL

#include "shared_light.hlsli"

static const int Light_None = -1;
static const int Light_Point = 0;
static const int Light_Directional = 1;
static const int Light_Diffuse_Rectangular = 2;

[[vk::binding(POINT_LIGHT_BINDING, LIGHT_SET_INDEX)]]
StructuredBuffer<Point_Light> point_lights;

[[vk::binding(DIRECTIONAL_LIGHT_BINDING, LIGHT_SET_INDEX)]]
StructuredBuffer<Directional_Light> directional_lights;

[[vk::binding(DIFFUSE_RECTANGULAR_LIGHT_BINDING, LIGHT_SET_INDEX)]]
StructuredBuffer<Diffuse_Rectangular_Light> diffuse_rectangular_lights;

#endif
