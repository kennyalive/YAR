#ifndef COMMON_HLSL
#define COMMON_HLSL

static const float Pi = 3.14159265358;
static const float Pi_Inv = 1.f / Pi;
static const float Shadow_Epsilon = 0.0001;

float srgb_encode(float c)
{
    if (c <= 0.0031308)
        return 12.92 * c;
    else
        return 1.055 * pow(c, 1.0 / 2.4) - 0.055;
}

float3 srgb_encode(float3 c)
{
    return float3(srgb_encode(c.r), srgb_encode(c.g), srgb_encode(c.b));
}

float2 barycentric_interpolate(float b1, float b2, float2 v0, float2 v1, float2 v2)
{
    return (1.0 - b1 - b2) * v0 + b1 * v1 + b2 * v2;
}

float3 barycentric_interpolate(float b1, float b2, float3 v0, float3 v1, float3 v2)
{
    return (1.0 - b1 - b2) * v0 + b1 * v1 + b2 * v2;
}

uint wang_hash(uint seed)
{
    seed = (seed ^ 61) ^ (seed >> 16);
    seed *= 9;
    seed = seed ^ (seed >> 4);
    seed *= 0x27d4eb2d;
    seed = seed ^ (seed >> 15);
    return seed;
}

// Xorshift algorithm from George Marsaglia's paper.
uint rand_xorshift(uint rng_state)
{
    rng_state ^= (rng_state << 13);
    rng_state ^= (rng_state >> 17);
    rng_state ^= (rng_state << 5);
    return rng_state;
}

// Ray Tracing Gems, chapter 6: A Fast and Robust Method for Avoiding Self-Intersection.
float3 offset_ray(float3 p, float3 n)
{
    static const float int_scale = 256.0;
    int3 of_i = int3(int(int_scale * n.x), int(int_scale * n.y), int(int_scale * n.z));

    float3 p_i;
    p_i.x = asfloat(asint(p.x) + ((p.x < 0) ? -of_i.x : of_i.x));
    p_i.y = asfloat(asint(p.y) + ((p.y < 0) ? -of_i.y : of_i.y));
    p_i.z = asfloat(asint(p.z) + ((p.z < 0) ? -of_i.z : of_i.z));

    static const float origin = 1.0 / 32.0;
    static const float float_scale = 1.0 / 65536.0;
    float3 p_adjusted;
    p_adjusted.x = abs(p.x) < origin ? p.x + float_scale * n.x : p_i.x;
    p_adjusted.y = abs(p.y) < origin ? p.y + float_scale * n.y : p_i.y;
    p_adjusted.z = abs(p.z) < origin ? p.z + float_scale * n.z : p_i.z;
    return p_adjusted;
}

#endif
