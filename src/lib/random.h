#pragma once

#include "pcg/pcg_basic.h"

// [0, 1)
inline float random_float(pcg32_random_t* rng) {
    union {
        float f;
        uint32_t i;
    } u;
    u.i = (pcg32_random_r(rng) >> 9) | 0x3f800000u;
    return u.f - 1.0f;
}
