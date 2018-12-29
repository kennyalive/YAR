#pragma once

#include "lib/vector.h"

inline Vector3 f_diffuse(Vector3 albedo) {
    return albedo * Pi_Inv;
}
