#ifndef COMMON_HLSL
#define COMMON_HLSL

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

#endif
