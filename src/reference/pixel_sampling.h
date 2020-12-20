#pragma once

// The pixel samplers generate well distributed set of samples needed to shade the entire pixel.
// 
// For each pixel sample the sampler generates 1d/2d multidimentional sample vectors.
// The values from the corresponding dimensions of the sample vectors are well distributed within a single pixel.
// It's recommended to use sample vectors as the source of pseudo-random numbers instead of using raw RNG instance.
//
// Also for each pixel sample it's possible to request 2d array of samples.
// The samples from corresponding arrays are well distributed within a single pixel.

#include "lib/vector.h"

struct RNG;

// The data that is shared between all Stratified_Pixel_Sampler instances. It is a Scene_Context state.
struct Stratified_Pixel_Sampler_Configuration {
    int x_pixel_sample_count = 0;
    int y_pixel_sample_count = 0;
    int sample_vector_1d_size = 0;
    int sample_vector_2d_size = 0;

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

    void init(int x_pixel_sample_count, int y_pixel_sample_count, int sample_vector_1d_size, int sample_vector_2d_size);

    // Registers 2d array of stratified samples distributed over [0, 1)^2.
    // Returns array2d_id to use in Stratified_Pixel_Sampler::get_array2d_samples
    int register_array2d_samples(int x_size, int y_size);
};

// Generates a set of samples for entire pixel.
struct Stratified_Pixel_Sampler {
    const Stratified_Pixel_Sampler_Configuration* config = nullptr;
    RNG* rng = nullptr;
    int current_sample_vector = 0;

    // Generated samples.
    std::vector<Vector2> image_plane_samples;

    std::vector<float>  samples_1d;
    int current_sample_1d = 0;

    std::vector<Vector2> samples_2d;
    int current_sample_2d = 0;

    std::vector<Vector2> array2d_samples; // [0..1)^2 samples for all registered 2d arrays for all pixel samples

    void init(const Stratified_Pixel_Sampler_Configuration* config, RNG* rng);

    // Generates samples for the next pixel and makes the first sample vector active.
    void next_pixel();

    // Makes the next sample vector active. Returns false is there are no sample vectors left.
    bool next_sample_vector();

    Vector2 get_image_plane_sample() const;
    const Vector2* get_array2d(int array2d_id) const;
    float get_next_1d_sample();
    Vector2 get_next_2d_sample();
};
