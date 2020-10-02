#pragma once

#include "lib/vector.h"

struct RNG;

struct Stratified_Pixel_Sampler {
    int x_pixel_samples = 0;
    int y_pixel_samples = 0;
    std::vector<Vector2> image_plane_samples;

    struct Light_Info {
        int x_light_samples;
        int y_light_samples;
        // Offset in light_samples array to a sequence of (light_samples * pixel_samples) samples.
        // The sequence starts with (x_light_samples * y_light_samples) light samples  for the first pixel sample,
        // followed by the same amount of light samples for the second pixel sample and so on.
        int first_sample_offset;
    };
    std::vector<Light_Info> light_infos; // Indexed by id returned from register_light()
    std::vector<Vector2> light_samples; // The [0..1)^2 samples for all registered lights

    void init(int x_pixel_samples, int y_pixel_samples);
    int register_light(int light_sample_count); // returns light_id
    void generate_pixel_samples(RNG& rng);

    int get_pixel_sample_count() const { return x_pixel_samples * y_pixel_samples; }
    Vector2 get_pixel_sample(int pixel_sample_index) const { return image_plane_samples[pixel_sample_index]; }
    const Vector2* get_light_samples(int pixel_sample_index, int light_id) const;
};
