layout(row_major) uniform;
layout(row_major) buffer;

const float Pi = 3.14159265358;
const float Pi_Inv = 1.f / Pi;
const float Shadow_Epsilon = 0.0001;

float srgb_encode(float c) {
    if (c <= 0.0031308f)
        return 12.92f * c;
    else
        return 1.055f * pow(c, 1.f/2.4f) - 0.055f;
}

vec3 srgb_encode(vec3 c) {
    return vec3(srgb_encode(c.r), srgb_encode(c.g), srgb_encode(c.b));
}

vec2 barycentric_interpolate(float b1, float b2, vec2 v0, vec2 v1, vec2 v2) {
    return (1.0 - b1 - b2)*v0 + b1*v1 + b2*v2;
}

vec3 barycentric_interpolate(float b1, float b2, vec3 v0, vec3 v1, vec3 v2) {
    return (1.0 - b1 - b2)*v0 + b1*v1 + b2*v2;
}

uint wang_hash(uint seed) {
    seed = (seed ^ 61) ^ (seed >> 16);
    seed *= 9;
    seed = seed ^ (seed >> 4);
    seed *= 0x27d4eb2d;
    seed = seed ^ (seed >> 15);
    return seed;
}

// Xorshift algorithm from George Marsaglia's paper.
uint rand_xorshift(uint rng_state) {
    rng_state ^= (rng_state << 13);
    rng_state ^= (rng_state >> 17);
    rng_state ^= (rng_state << 5);
    return rng_state;
}

// Ray Tracing Gems, chapter 6: A Fast and Robust Method for Avoiding Self-Intersection.
vec3 offset_ray(vec3 p, vec3 n) {
    const float int_scale = 256.0;
    ivec3 of_i = ivec3(int_scale * n.x, int_scale * n.y, int_scale * n.z);
   
    vec3 p_i;
    p_i.x = intBitsToFloat(floatBitsToInt(p.x) + ((p.x < 0) ? -of_i.x : of_i.x));
    p_i.y = intBitsToFloat(floatBitsToInt(p.y) + ((p.y < 0) ? -of_i.y : of_i.y));
    p_i.z = intBitsToFloat(floatBitsToInt(p.z) + ((p.z < 0) ? -of_i.z : of_i.z));

    const float origin = 1.0/32.0;
    const float float_scale = 1.0/65536.0;
    vec3 p_adjusted;
    p_adjusted.x = abs(p.x) < origin ? p.x + float_scale*n.x : p_i.x;
    p_adjusted.y = abs(p.y) < origin ? p.y + float_scale*n.y : p_i.y;
    p_adjusted.z = abs(p.z) < origin ? p.z + float_scale*n.z : p_i.z;
    return p_adjusted;
}
