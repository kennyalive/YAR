#ifndef RT_UTILS_HLSL
#define RT_UTILS_HLSL

struct Attribs {
    float2 barycentrics;
};

struct Ray_Payload {
    float3 color;
};

struct Shadow_Ray_Payload {
    float shadow_factor; // 0.0 or 1.0
};

struct Vertex {
    float3 p;
    float3 n;
    float2 uv;
};

struct Ray {
    float3 origin;
    float3 dir;
};

#ifdef RGEN_SHADER
float3 get_direction(float2 film_position)
{
    const float2 dispatch_dims = float2(DispatchRaysDimensions().xy);
    float2 uv = 2.0 * (film_position / dispatch_dims) - 1.0;
    float aspect_ratio = float(dispatch_dims.x) / float(dispatch_dims.y);

    float right = uv.x *  aspect_ratio * push_constants.tan_fovy_over_2;
    float up = -uv.y * push_constants.tan_fovy_over_2;

    // If Z is up then we are looking along Y axis.
    // If Y is up then we are looking along -Z axis.
    float3 direction;
    direction.x = right;
    direction.y = bool(push_constants.z_is_up) ? 1.f : up;
    direction.z = bool(push_constants.z_is_up) ? up : -1.f;
    return normalize(direction);
}

Ray generate_ray(float3x4 camera_to_world, float2 film_position)
{
    Ray ray;
    ray.origin  = camera_to_world._m03_m13_m23;
    ray.dir     = mul(camera_to_world, float4(get_direction(film_position), 0));
    return ray;
}
#endif // RGEN_SHADER

#endif
