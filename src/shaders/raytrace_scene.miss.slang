import common;
import raytrace_types;

[shader("miss")]
void main(inout Ray_Payload payload)
{
    payload.color = float3(0, 0, 0);
}
