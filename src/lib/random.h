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

    // Based on Daniel Lemire's blog post article:
    // https://lemire.me/blog/2016/06/30/fast-random-shuffling/
    //
    // Returns value from [0, bound) interval
    uint32_t get_bounded_uint(uint32_t bound) {
        uint64_t random32bit = get_uint();
        uint64_t multiresult = random32bit * bound;
        uint32_t leftover = (uint32_t) multiresult;
        if (leftover < bound ) {
            uint32_t threshold = (~bound + 1) % bound;
            while (leftover < threshold) {
                random32bit =  get_uint();
                multiresult = random32bit * bound;
                leftover = (uint32_t) multiresult;
            }
        }
        return multiresult >> 32;
    }

    // Based on Daniel Lemire's blog post article:
    // https://lemire.me/blog/2016/06/30/fast-random-shuffling/
    //
    // Bias is small if 'bound' is small compared to 2^32.
    // For example,
    //      for bound == 4 and ~1 billion function calls the rng did not reach state that introduces bias.
    //      for bound == 16 and ~1 billion function calls we had around 15 rng states that introduce bias.
    //      NOTE: 'The state that introduces bias' corresponds to the situation when 'if(leftover < bound )' in
    //      get_bounded_uint() evaluates to true.
    //
    // Returns value from [0, bound) interval
    uint32_t get_bounded_uint_fast_and_biased(uint32_t bound) {
        uint64_t random32bit = get_uint();
        uint64_t multiresult = random32bit * bound;
        return multiresult >> 32;
    }

    // [0, 1)
    float get_float() {
        uint32_t i = (pcg32_random_r(&pcg_state) >> 9) | 0x3f800000u;
        float f;
        memcpy(&f, &i, sizeof(uint32_t));
        return f - 1.0f;
    }

    // [0, 1)^2
    Vector2 get_vector2() {
        float x = get_float();
        float y = get_float();
        return {x, y};
    }
};
