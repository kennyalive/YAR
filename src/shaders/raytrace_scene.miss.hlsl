#include "common.hlsli"
#include "rt_utils.hlsli"

[shader("miss")]
void main(inout Ray_Payload payload)
{
    payload.color = srgb_encode(float3(0, 0, 0));
}
