#pragma once

#include <algorithm>
#include <cstdint>

class RNG {
public:
    RNG() {
        static const uint64_t Init_State = 0x853c49e6748fea9bULL;
        static const uint64_t Init_Inc = 0xda3e39cb94b95bdbULL;

        state = Init_State;
        inc = Init_Inc;
    }

    uint32_t random_uint32() {
        uint64_t oldstate = state;
        state = oldstate * 6364136223846793005ULL + inc;
        uint32_t xorshifted = static_cast<uint32_t>(((oldstate >> 18u) ^ oldstate) >> 27u);
        uint32_t rot = oldstate >> 59u;
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1) & 31));
    }

    float random_float() {
        static const float Float_One_Minus_Epsilon = 0.99999994f;

        float r = random_uint32() * 2.3283064365e-10f;
        return std::min(r, Float_One_Minus_Epsilon);
    }

    float random_from_range(float a, float b) {
        return a + (b - a) * random_float();
    }

private:
    uint64_t state; // RNG state.  All values are possible.
    uint64_t inc;   // Controls which RNG sequence (stream) is selected. Must *always* be odd.
};
