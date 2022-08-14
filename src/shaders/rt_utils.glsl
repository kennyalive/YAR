struct Ray_Payload {
    vec3 color;
};

struct Shadow_Ray_Payload {
    float shadow_factor; // 0 or 1
};

struct Vertex {
    vec3 p;
    vec3 n;
    vec2 uv;
};

struct Ray {
    vec3 origin;
    vec3 dir;
};

#ifdef RGEN_SHADER
vec3 get_direction(vec2 film_position) {
    vec2 uv = 2.0 * (film_position / vec2(gl_LaunchSizeEXT.xy)) - 1.0;
    float aspect_ratio = float(gl_LaunchSizeEXT.x) / float(gl_LaunchSizeEXT.y);

    float right = uv.x *  aspect_ratio * tan_fovy_over_2;
    float up = -uv.y * tan_fovy_over_2;

    // If Z is up then we are looking along Y axis.
    // If Y is up then we are looking along -Z axis.
    vec3 direction;
    direction.x = right;
    direction.y = bool(z_is_up) ? 1.f : up;
    direction.z = bool(z_is_up) ? up : -1.f;
    return normalize(direction);
}

Ray generate_ray(mat4x3 camera_to_world, vec2 film_position) {
    Ray ray;
    ray.origin  = camera_to_world[3];
    ray.dir     = camera_to_world * vec4(get_direction(film_position), 0);
    ray.rx_dir  = camera_to_world * vec4(get_direction(vec2(film_position.x + 1.f, film_position.y)), 0);
    ray.ry_dir  = camera_to_world * vec4(get_direction(vec2(film_position.x, film_position.y + 1.f)), 0);
    return ray;
}
#endif // RGEN_SHADER

