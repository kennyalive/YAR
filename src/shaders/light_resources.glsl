#ifndef LIGHT_RESOURCES_GLSL
#define LIGHT_RESOURCES_GLSL

#include "shared_main.h"
#include "shared_light.h"

const int Light_None = 0;
const int Light_Point = 1;
const int Light_Diffuse_Rectangular = 2;

layout(std430, set=LIGHT_SET_INDEX, binding=POINT_LIGHT_BINDING)
readonly buffer Point_Light_Buffer {
    Point_Light point_lights[];
};

layout(std430, set=LIGHT_SET_INDEX, binding=DIFFUSE_RECTANGULAR_LIGHT_BINDING)
readonly buffer Diffuse_Rectangular_Light_Buffer {
    Diffuse_Rectangular_Light diffuse_rectangular_lights[];
};

#endif // LIGHT_RESOURCES_GLSL
