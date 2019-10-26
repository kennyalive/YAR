#include "shared_light.h"

layout(std430, set=LIGHT_SET_INDEX, binding=POINT_LIGHT_BINDING)
readonly buffer Point_Light_Buffer {
    Point_Light point_lights[];
};

layout(std430, set=LIGHT_SET_INDEX, binding=DIFFUSE_RECTANGULAR_LIGHT_BINDING)
readonly buffer Diffuse_Rectangular_Light_Buffer {
    Diffuse_Rectangular_Light diffuse_rectangular_lights[];
};

