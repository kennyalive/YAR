#pragma once

// A pixel sampler generates well distributed set of samples needed to shade the entire pixel.
//
// For each pixel sample the sampler generates 1d and 2d multidimentional sample vectors.
// Mmultiple dimensions for each sample are needed to account for multiple bounces. Also
// for a specific bounce we need a set of random value for difference purposes.
//
// The values from the corresponding dimensions of the sample vectors are well distributed within a single pixel.
// It's recommended to use sample vectors as the source of pseudo-random numbers instead of using raw RNG instance.
//
// Also for each pixel sample it's possible to request 1d/2d array of samples.
// The samples from corresponding arrays are well distributed within a single pixel.
// This is used by the basic direct lighting renderer (and not used by the path tracing).

#include "lib/vector.h"

struct RNG;

// Data shared between all Stratified_Pixel_Sampler instances.
struct Stratified_Pixel_Sampler_Configuration {
    int x_pixel_sample_count = 0;
    int y_pixel_sample_count = 0;
    int sample_vector_1d_size = 0;
    int sample_vector_2d_size = 0;

    void init(int x_pixel_sample_count, int y_pixel_sample_count, int sample_vector_1d_size, int sample_vector_2d_size);
    int get_samples_per_pixel() const { return x_pixel_sample_count * y_pixel_sample_count; }

    //
    // Arrays of samples for basic direct lighting renderer.
    //
    struct Array2D_Info {
        // For each registered 2d array and for each pixel sample we generate (x_size, y_size) grid of [0, 1)^2 samples.
        // All grids definied by the same Array2D_Info but generated for different pixel samples are well distributed
        // with repsect to each other. This is achieved by additional level of stratification.
        int x_size = 0;
        int y_size = 0;
        // Start position of samples in Stratified_Pixel_Sampler::array2d_samples buffer.
        // The sequence starts with (x_size * y_size) samples for the first pixel sample,
        // followed by the same amount of samples for the second pixel sample, etc.
        int first_sample_offset = -1;
    };
    std::vector<Array2D_Info> array2d_infos; // Indexed by id returned from register_array2d_samples()
    int array2d_samples_per_pixel = 0;
    // Registers 2d array of stratified samples distributed over [0, 1)^2.
    // Returns id to use in Stratified_Pixel_Sampler::get_array2d.
    int register_array2d_samples(int x_size, int y_size);

    struct Array1D_Info {
        int size = 0;
        int first_sample_offset = -1;
    };
    std::vector<Array1D_Info> array1d_infos;
    int array1d_samples_per_pixel = 0;
    // Registers 1d array of stratified samples distributed over [0, 1).
    // Returns id to use in Stratified_Pixel_Sampler::get_array1d.
    int register_array1d_samples(int size);
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

    void init(const Stratified_Pixel_Sampler_Configuration* config, RNG* rng);

    // Generates samples for the next pixel and makes the first sample vector active.
    void next_pixel();

    // Makes the next sample vector active. Returns false is there are no sample vectors left.
    bool next_sample_vector();

    Vector2 get_image_plane_sample() const;
    float get_next_1d_sample();
    Vector2 get_next_2d_sample();

    //
    // Arrays of samples for basic direct lighting renderer.
    //
    std::vector<Vector2> array2d_samples; // [0..1)^2 samples for all registered 2d arrays for all pixel samples
    std::vector<float> array1d_samples; // [0..1) samples for all registered 1d arrays for all pixel samples
    const Vector2* get_array2d(int array2d_id) const;
    const float* get_array1d(int array1d_id) const;
};
