module generate_ray;

import raytrace_types;

float3 get_direction(float2 film_position, float tan_fovy_over_2, uint z_is_up)
{
    const float2 dispatch_dims = float2(DispatchRaysDimensions().xy);
    float2 uv = 2.0 * (film_position / dispatch_dims) - 1.0;
    float aspect_ratio = float(dispatch_dims.x) / float(dispatch_dims.y);

    float right = uv.x *  aspect_ratio * tan_fovy_over_2;
    float up = -uv.y * tan_fovy_over_2;

    // If Z is up then we are looking along Y axis.
    // If Y is up then we are looking along -Z axis.
    float3 direction;
    direction.x = right;
    direction.y = bool(z_is_up) ? 1.f : up;
    direction.z = bool(z_is_up) ? up : -1.f;
    return normalize(direction);
}

public Ray generate_ray(float3x4 camera_to_world, float2 film_position, float tan_fovy_over_2, uint z_is_up)
{
    Ray ray;
    ray.origin  = camera_to_world._m03_m13_m23;
    ray.dir     = mul(camera_to_world, float4(get_direction(film_position, /*0.4, 0*/ tan_fovy_over_2, z_is_up), 0));
    return ray;
}
