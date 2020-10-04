#include "std.h"
#include "lib/common.h"
#include "pixel_sampling.h"

#include "sampling.h"

#include "lib/math.h"
#include "lib/random.h"

void Stratified_Pixel_Sampler_Configuration::init(int x_pixel_samples, int y_pixel_samples) {
    this->x_pixel_samples = x_pixel_samples;
    this->y_pixel_samples = y_pixel_samples;
}

int Stratified_Pixel_Sampler_Configuration::register_array2d_samples(int x_size, int y_size) {
    Array2D_Info info;
    info.x_size = x_size;
    info.y_size = y_size;
    info.first_sample_offset = array2d_samples_per_pixel;
    array2d_infos.push_back(info);

    array2d_samples_per_pixel += (x_size * y_size) * (x_pixel_samples * y_pixel_samples);

    return (int)array2d_infos.size() - 1;
}

void Stratified_Pixel_Sampler::init(const Stratified_Pixel_Sampler_Configuration* config) {
    this->config = config;
    image_plane_samples.resize(config->x_pixel_samples * config->y_pixel_samples);
    array2d_samples.resize(config->array2d_samples_per_pixel);
}

void Stratified_Pixel_Sampler::generate_samples(RNG& rng) {
    int pixel_sample_count = config->x_pixel_samples * config->y_pixel_samples;

    // Film plane samples.
    generate_stratified_sequence_2d(rng, config->x_pixel_samples, config->y_pixel_samples, image_plane_samples.data());

    // Array 2D samples.
    for (const auto& array_info : config->array2d_infos) {
        int array_sample_count = array_info.x_size * array_info.y_size;

        // sequence of array_sample_count grids of stratified samples of size (x_pixel_samples, y_pixel_samples)
        std::vector<Vector2> stratified_grids(array_sample_count * pixel_sample_count);

        for (int i = 0; i < array_sample_count; i++) {
            Vector2* grid = &stratified_grids[i * pixel_sample_count];
            generate_stratified_sequence_2d(rng, config->x_pixel_samples, config->y_pixel_samples, grid);
            shuffle(grid, grid + pixel_sample_count, rng);
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
}

const Vector2* Stratified_Pixel_Sampler::get_array2d(int pixel_sample_index, int array2d_id) const {
    ASSERT(array2d_id >= 0);
    const auto& info = config->array2d_infos[array2d_id];
    return &array2d_samples[info.first_sample_offset + pixel_sample_index * info.x_size * info.y_size];
}
