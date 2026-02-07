#include "rt_utils.slangi"

[shader("closesthit")]
void main(inout Shadow_Ray_Payload payload, in Attribs attribs)
{
    payload.shadow_factor = 0.0;
}
