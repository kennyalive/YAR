#include "std.h"
#include "lib/common.h"
#include "pixel_sampling.h"

#include "sampling.h"

#include "lib/math.h"
#include "lib/random.h"

void Stratified_Pixel_Sampler_Configuration::init(
    int x_pixel_sample_count, int y_pixel_sample_count, 
    int sample_vector_1d_size, int sample_vector_2d_size)
{
    this->x_pixel_sample_count = x_pixel_sample_count;
    this->y_pixel_sample_count = y_pixel_sample_count;
    this->sample_vector_1d_size = sample_vector_1d_size;
    this->sample_vector_2d_size = sample_vector_2d_size;
}

int Stratified_Pixel_Sampler_Configuration::register_array2d_samples(int x_size, int y_size)
{
    Array2D_Info info;
    info.x_size = x_size;
    info.y_size = y_size;
    info.first_sample_offset = array2d_samples_per_pixel;
    array2d_infos.push_back(info);
    array2d_samples_per_pixel += (x_size * y_size) * (x_pixel_sample_count * y_pixel_sample_count);
    return (int)array2d_infos.size() - 1;
}

int Stratified_Pixel_Sampler_Configuration::register_array1d_samples(int size)
{
    Array1D_Info info;
    info.size = size;
    info.first_sample_offset = array1d_samples_per_pixel;
    array1d_infos.push_back(info);
    array1d_samples_per_pixel += size * (x_pixel_sample_count * y_pixel_sample_count);
    return (int)array1d_infos.size() - 1;
}

void Stratified_Pixel_Sampler::init(const Stratified_Pixel_Sampler_Configuration* config, RNG* rng)
{
    this->config = config;
    this->rng = rng;

    int pixel_sample_count = config->x_pixel_sample_count * config->y_pixel_sample_count;
    image_plane_samples.resize(pixel_sample_count);
    samples_1d.resize(config->sample_vector_1d_size * pixel_sample_count);
    samples_2d.resize(config->sample_vector_2d_size * pixel_sample_count);
    array2d_samples.resize(config->array2d_samples_per_pixel);
    array1d_samples.resize(config->array1d_samples_per_pixel);
}

void Stratified_Pixel_Sampler::next_pixel()
{
    current_sample_vector = 0;

    // Generate film plane samples.
    generate_stratified_sequence_2d(*rng, config->x_pixel_sample_count, config->y_pixel_sample_count, image_plane_samples.data());

    int pixel_sample_count = config->x_pixel_sample_count * config->y_pixel_sample_count;

    // Generate 1d samples.
    {
        std::vector<float> samples(pixel_sample_count);
        for (int i = 0; i < config->sample_vector_1d_size; i++) {
            generate_stratified_sequence_1d(*rng, pixel_sample_count, samples.data());
            shuffle(samples.data(), pixel_sample_count, *rng);
            for (int k = 0; k < pixel_sample_count; k++) {
                samples_1d[k * config->sample_vector_1d_size + i] = samples[k];
            }
        }
    }

    // Generate 2d samples.
    {
        std::vector<Vector2> samples(pixel_sample_count);
        for (int i = 0; i < config->sample_vector_2d_size; i++) {
            generate_stratified_sequence_2d(*rng, config->x_pixel_sample_count, config->y_pixel_sample_count, samples.data());
            shuffle(samples.data(), pixel_sample_count, *rng);
            for (int k = 0; k < pixel_sample_count; k++) {
                samples_2d[k * config->sample_vector_2d_size + i] = samples[k];
            }
        }
    }

    // Generate 2d array samples.
    for (const auto& array_info : config->array2d_infos) {
        int array_sample_count = array_info.x_size * array_info.y_size;

        // sequence of array_sample_count grids of stratified samples of size (x_pixel_samples, y_pixel_samples)
        std::vector<Vector2> stratified_grids(array_sample_count * pixel_sample_count);
        for (int i = 0; i < array_sample_count; i++) {
            Vector2* grid = &stratified_grids[i * pixel_sample_count];
            generate_stratified_sequence_2d(*rng, config->x_pixel_sample_count, config->y_pixel_sample_count, grid);
            shuffle(grid, pixel_sample_count, *rng);
        }
        float dx_array = 1.f / float(array_info.x_size);
        float dy_array = 1.f / float(array_info.y_size);
        Vector2* s = &array2d_samples[array_info.first_sample_offset];

        for (int i = 0; i < pixel_sample_count; i++) {
            // Get the first array2d sample for the current pixel sample 'i'.
            // This sample is in the first stratified grid in position defined by pixel sample index.
            Vector2* u  = &stratified_grids[i];

            for (int k = 0; k < array_sample_count; k++) {
                int x = k % array_info.x_size;
                int y = k / array_info.x_size;

                float sx = std::min(float(x + u->x) * dx_array, One_Minus_Epsilon);
                float sy = std::min(float(y + u->y) * dy_array, One_Minus_Epsilon);
                *s++ = Vector2(sx, sy);

                // Go to the next array2d sample for the current pixel sample 'i' by jumping
                // to the same location in the next stratified grid.
                u += pixel_sample_count;
            }
        }
    }

    // Generate 1d array samples (computations are analogous to 2d array case).
    for (const auto& array_info : config->array1d_infos) {
        // sequence of array_info.size grids of stratified samples  of size (x_pixel_samples, y_pixel_samples)
        std::vector<float> stratified_grids(array_info.size * pixel_sample_count);
        for (int i = 0; i < array_info.size; i++) {
            float* grid = &stratified_grids[i * pixel_sample_count];
            generate_stratified_sequence_1d(*rng, pixel_sample_count, grid);
            shuffle(grid, pixel_sample_count, *rng);
        }
        float dx_array = 1.f / float(array_info.size);
        float* s = &array1d_samples[array_info.first_sample_offset];
        for (int i = 0; i < pixel_sample_count; i++) {
            float* u = &stratified_grids[i];
            for (int x = 0; x < array_info.size; x++) {
                *s++ = std::min(float(x + *u) * dx_array, One_Minus_Epsilon);
                u += pixel_sample_count;
            }
        }
    }
}

bool Stratified_Pixel_Sampler::next_sample_vector()
{
    int pixel_sample_count = config->x_pixel_sample_count * config->y_pixel_sample_count;
    if (current_sample_vector < pixel_sample_count) {
        current_sample_vector++;
        current_sample_1d = 0;
        current_sample_2d = 0;
    }
    return current_sample_vector < pixel_sample_count;
}

Vector2 Stratified_Pixel_Sampler::get_image_plane_sample() const
{
    return image_plane_samples[current_sample_vector];
}

float Stratified_Pixel_Sampler::get_next_1d_sample()
{
    if (current_sample_1d < config->sample_vector_1d_size)
        return samples_1d[current_sample_vector * config->sample_vector_1d_size + current_sample_1d++];
    else
        return rng->get_float();
}

Vector2 Stratified_Pixel_Sampler::get_next_2d_sample()
{
    if (current_sample_2d < config->sample_vector_2d_size)
        return samples_2d[current_sample_vector * config->sample_vector_2d_size + current_sample_2d++];
    else
        return rng->get_vector2();
}

const Vector2* Stratified_Pixel_Sampler::get_array2d(int array2d_id) const
{
    ASSERT(array2d_id >= 0);
    const auto& info = config->array2d_infos[array2d_id];
    return &array2d_samples[info.first_sample_offset + current_sample_vector * info.x_size * info.y_size];
}

const float* Stratified_Pixel_Sampler::get_array1d(int array1d_id) const
{
    ASSERT(array1d_id >= 0);
    const auto& info = config->array1d_infos[array1d_id];
    return &array1d_samples[info.first_sample_offset + current_sample_vector * info.size];
}
