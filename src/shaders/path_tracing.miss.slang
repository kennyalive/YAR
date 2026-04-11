import common;
import raytrace_types;

[shader("miss")]
void main(inout Path_Payload payload)
{
    payload.hit = false;
    payload.color = srgb_encode(float3(0, 1, 0));
}
