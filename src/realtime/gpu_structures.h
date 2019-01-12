#pragma once

#include "lib/vector.h"

struct GPU_Point_Light {
    Vector3 position;
    float   padding0;
    Vector3 intensity;
    float   padding1;
};
