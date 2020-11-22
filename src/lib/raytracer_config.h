#pragma once

struct Raytracer_Config {
    enum class Rendering_Algorithm {
        direct_lighting,
        path_tracer
    };
    Rendering_Algorithm rendering_algorithm = Rendering_Algorithm::path_tracer;

    // The path length is defined as the number of line segments that connect camera and light source for
    // specific light path. This limit is used by the algorithms that try to compute rendering equation
    // integral (for example, path tracing). It is also used in direct lighting rendering when we have
    // longer light passes due to specular reflection and refraction.
    //
    // 0 - light is not emitted, complete darkness
    // 1 - only emmited light
    // 2 - direct lighting
    // 3 - first bounce of indirect lighting
    // 4 - second bound of indirect lighting
    // ...
    int max_path_length = 100;

    enum class Pixel_Filter_Type {
        box,
        gaussian
    };
    Pixel_Filter_Type pixel_filter_type = Pixel_Filter_Type::box;
    float pixel_filter_radius = 0.5f;
    float pixel_filter_alpha = 2.f; // used by gaussian filter
};
