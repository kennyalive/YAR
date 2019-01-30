#pragma once

#include "lib/color.h"
#include "lib/vector.h"

struct GPU_Point_Light {
    Vector3     position;
    float       padding0;
    ColorRGB    intensity;
    float       padding1;
};
