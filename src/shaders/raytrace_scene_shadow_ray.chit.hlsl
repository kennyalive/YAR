#include "rt_utils.hlsli"

[shader("closesthit")]
void main(inout Shadow_Ray_Payload payload)
{
    payload.shadow_factor = 0.0;
}
