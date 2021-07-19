#pragma once

struct Raytracer_Config {
    enum class Rendering_Algorithm {
        direct_lighting,
        path_tracer
    };
    Rendering_Algorithm rendering_algorithm = Rendering_Algorithm::path_tracer;

    // This constant limits how many times the light can bounce off of the surfaces.
    // It is used in the algorithms that try to solve rendering equation and also in
    // direct lighting to limit the path length when specular surfaces are present.
    //
    // Here is the meaning of some specific numbers of bounce count:
    // 0 - light does not interact with the surfaces, we can only see the emitted light that goes directly into the camera
    // 1 - single bounce (direct lighting)
    // 2 - first bounce of indirect lighting
    // 3 - second bound of indirect lighting
    int max_light_bounces = 32;

    enum class Pixel_Filter_Type {
        box,
        gaussian,
        triangle
    };
    Pixel_Filter_Type pixel_filter_type = Pixel_Filter_Type::box;
    float pixel_filter_radius = 0.5f;
    float pixel_filter_alpha = 2.f; // used by gaussian filter

    int x_pixel_sample_count = 1;
    int y_pixel_sample_count = 1;
};
