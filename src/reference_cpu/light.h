#pragma once

#include "spectrum.h"
#include "lib/vector.h"

struct Point_Light {
    Vector3  world_position;
    RGB     intensity;
};
