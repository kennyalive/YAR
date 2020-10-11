#pragma once

#include "vector.h"

#include "pcg/pcg_basic.h"

struct RNG {
    pcg32_random_t pcg_state;

    void init(uint64_t init_state, uint64_t stream_id) {
        pcg32_srandom_r(&pcg_state, init_state, stream_id);
    }

    // uniformly distributed 32-bit number
    uint32_t get_uint() {
        return pcg32_random_r(&pcg_state);
    }

    // [0, bound)
    uint32_t get_bounded_uint(uint32_t bound) {
        return pcg32_boundedrand_r(&pcg_state, bound);
    }

    // [0, 1)
    float get_float() {
        union {
            float f;
            uint32_t i;
        } u;
        u.i = (pcg32_random_r(&pcg_state) >> 9) | 0x3f800000u;
        return u.f - 1.0f;
    }

    // [0, 1)^2
    Vector2 get_vector2() {
        float x = get_float();
        float y = get_float();
        return {x, y};
    }
};
