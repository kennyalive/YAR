#include "common.slangi"
#include "rt_utils.slangi"

[shader("miss")]
void main(inout Ray_Payload payload)
{
    payload.color = srgb_encode(float3(0, 0, 0));
}
