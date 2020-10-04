#pragma once

// The pixel samplers try to generate well distributed set of samples needed to shade the entire pixel.
// Internally the pixel sampler may generate more samples than needed for a single pixel but the interface
// exposes samples only for the current pixel.
// 
// The generated per-pixel samples it's not just a film plane positions. For each film position it's possible
// to request 2d array of samples. It can be used as pseudo-random sequence that helps to generate samples on
// the area light sources. Also, if any other algorithm requires pseudo-random numbers that are well distribued
// within a single pixel, then it's a good idea to use pixel sampler instead of RNG instance.

#include "lib/vector.h"

struct RNG;

// The data that is shared between all Stratified_Pixel_Sampler instances.
// It is Scene_Context state.
struct Stratified_Pixel_Sampler_Configuration {
    int x_pixel_samples = 0;
    int y_pixel_samples = 0;

    struct Array2D_Info {
        // For each registered 2d array and for each pixel sample we generate (x_size, y_size) grid of [0, 1)^2 samples.
        // All corresponding grid within single pixel (i.e. grid definied by the same Array2D_Info but generated for 
        // different pixel samples) are well distributed with repsect to each other. This is achieved by additional level
        // of stratification.
        int x_size = 0;
        int y_size = 0;

        // Start position of samples in Stratified_Pixel_Sampler::array2d_samples buffer.
        // The sequence starts with (x_size * y_size) samples for the first pixel sample,
        // followed by the same amount of samples for the second pixel sample, etc.
        int first_sample_offset = -1;
    };
    std::vector<Array2D_Info> array2d_infos; // Indexed by id returned from register_array2d_samples()
    int array2d_samples_per_pixel = 0;

    void init(int x_pixel_samples, int y_pixel_samples);

    // Registers 2d array of stratified samples distributed over [0, 1)^2.
    // Returns array2d_id to use in Stratified_Pixel_Sampler::get_array2d_samples
    int register_array2d_samples(int x_size, int y_size);
};

// Generates a set of sample for entire pixel.
// It is Thread_Context state.
struct Stratified_Pixel_Sampler {
    // each pixel sampler instance references to the same config object
    const Stratified_Pixel_Sampler_Configuration* config = nullptr;

    // Generated samples.
    std::vector<Vector2> image_plane_samples;
    std::vector<Vector2> array2d_samples; // [0..1)^2 samples for all registered 2d arrays for all pixel samples

    void init(const Stratified_Pixel_Sampler_Configuration* config);

    // Generates registered samples for entire pixel.
    // Should be called each time we start working with new pixel.
    void generate_samples(RNG& rng);

    int get_pixel_sample_count() const { return config->x_pixel_samples * config->y_pixel_samples; }
    Vector2 get_image_plane_position(int pixel_sample_index) const { return image_plane_samples[pixel_sample_index]; }
    const Vector2* get_array2d(int pixel_sample_index, int array2d_id) const;
};
