#include "std.h"
#include "lib/common.h"
#include "pixel_sampling.h"

#include "sampling.h"

#include "lib/math.h"
#include "lib/random.h"

void Stratified_Pixel_Sampler::init(int x_pixel_samples, int y_pixel_samples) {
    this->x_pixel_samples = x_pixel_samples;
    this->y_pixel_samples = y_pixel_samples;
    image_plane_samples.resize(x_pixel_samples * y_pixel_samples);
}

int Stratified_Pixel_Sampler::register_light(int light_sample_count) {
    int k = (int)std::ceil(std::sqrt(light_sample_count));
    ASSERT(k*k >= light_sample_count);
    ASSERT((k-1)*(k-1) < light_sample_count);

    Light_Info info;
    info.x_light_samples = k;
    info.y_light_samples = k;
    info.first_sample_offset = (int)light_samples.size();
    light_infos.push_back(info);

    int light_samples_per_pixel = (info.x_light_samples * info.y_light_samples) * (x_pixel_samples * y_pixel_samples);
    light_samples.resize(light_samples.size() + light_samples_per_pixel);
    return (int)light_infos.size() - 1;
}

void Stratified_Pixel_Sampler::generate_pixel_samples(RNG& rng) {
    int pixel_sample_count = x_pixel_samples * y_pixel_samples;

    // Film plane samples.
    generate_stratified_sequence_2d(rng, x_pixel_samples, y_pixel_samples, image_plane_samples.data());

    // Light samples.
    for (const Light_Info& light_info : light_infos) {
        int light_sample_count = light_info.x_light_samples * light_info.y_light_samples;

        // sequence of light_sample_count grids of stratified samples of size (x_pixel_samples, y_pixel_samples)
        std::vector<Vector2> stratified_grids(light_sample_count * pixel_sample_count);

        for (int i = 0; i < light_sample_count; i++) {
            Vector2* grid = &stratified_grids[i * pixel_sample_count];
            generate_stratified_sequence_2d(rng, x_pixel_samples, y_pixel_samples, grid);
            shuffle(grid, grid + pixel_sample_count, rng);
        }

        float dx_light = 1.f / float(light_info.x_light_samples);
        float dy_light = 1.f / float(light_info.y_light_samples);
        Vector2* s = &light_samples[light_info.first_sample_offset];

        for (int i = 0; i < pixel_sample_count; i++) {

            // Get the first light sample for the current pixel sample 'i'.
            // This sample is in the first stratified grid in position defined by pixel sample index.
            Vector2* u  = &stratified_grids[i];

            for (int k = 0; k < light_sample_count; k++) {
                int x = k % light_info.x_light_samples;
                int y = k / light_info.x_light_samples;

                float sx = std::min(float(x + u->x) * dx_light, One_Minus_Epsilon);
                float sy = std::min(float(y + u->y) * dy_light, One_Minus_Epsilon);
                *s++ = Vector2(sx, sy);

                // Go to the next light sample for the current pixel sample 'i' by jumping
                // to the same location in the next stratified grid.
                u += pixel_sample_count; 
            }
        }
    }
}

const Vector2* Stratified_Pixel_Sampler::get_light_samples(int pixel_sample_index, int light_index) const {
    const Light_Info& info = light_infos[light_index];
    return &light_samples[info.first_sample_offset + pixel_sample_index * info.x_light_samples * info.y_light_samples];
}
