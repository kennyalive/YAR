#include "std.h"
#include "lib/common.h"

#include "sampling.h"

#include "lib/math.h"
#include "lib/random.h"
#include "lib/vector.h"

void test_uniform_sphere_sampling() {
    // the number of poins to sample on a sphere
    const int n = 25'000'000;

    // max allowed difference in % between estimated number of samples on the sphere sector and the actual number
    const float error_tolerance_percentage = 0.15f;

    // max percentage of sphere sectors for which estimates is not within error_tolerance_percentage
    const float fail_threshold = 0.01f; 

    const int sector_angle_degrees = 4;
    const float sector_angle = radians((float)sector_angle_degrees);
    const int theta_slice_count = 180 / sector_angle_degrees;
    const int phi_slice_count = 360 / sector_angle_degrees;

    printf("Testing uniform sphere sampling...\n");
    printf("N_Sphere_Samples = %d, N_Sphere_Sectors = %d\n", n, theta_slice_count * phi_slice_count);
    printf("Error tolerance (actual vs estimated sample count): %.1f%%\n", error_tolerance_percentage * 100.f);

    std::vector<std::vector<int>> sphere_sectors;
    sphere_sectors.resize(theta_slice_count);
    for (auto& theta_slice : sphere_sectors)
        theta_slice.resize(phi_slice_count, 0);

    RNG rng;
    rng.init(0, 0x12345);

    Timestamp t;
    int failed_estimate_count = 0;
    for (int i = 0; i < n; i++) {
        Vector2 u = rng.get_vector2();
        Vector3 p = sample_sphere_uniform(u);

        ASSERT(std::abs(p.z) <= 1.f);
        float theta = std::acos(p.z);
        theta = std::clamp(theta, 0.f, Pi);
        int theta_slice = std::clamp(int((theta / Pi) * theta_slice_count), 0, theta_slice_count - 1);

        float phi = std::atan2(p.y, p.x);
        if (phi < 0)
            phi += Pi2;
        phi = std::clamp(phi, 0.f, Pi2);
        int phi_slice = std::clamp(int((phi / Pi2) * phi_slice_count), 0, phi_slice_count - 1);

        sphere_sectors[theta_slice][phi_slice]++;
    }

    for (int theta_slice = 0; theta_slice < theta_slice_count; theta_slice++) {
        for (int phi_slice = 0; phi_slice < phi_slice_count; phi_slice++) {
            float theta_start = radians(float(theta_slice * sector_angle_degrees));
            float phi_start = radians(float(phi_slice * sector_angle_degrees));

            // Area = (phi2 - phi1) * (cos(theta1) - cos(theta2))
            float area = sector_angle * (std::cos(theta_start) - std::cos(theta_start + sector_angle));
            float estimated_sample_count = area / (4 * Pi) * n;

            float sample_count = (float)sphere_sectors[theta_slice][phi_slice];

            float error = std::abs(estimated_sample_count - sample_count) / estimated_sample_count;

            if (error > error_tolerance_percentage)
                failed_estimate_count++;
        }
    }
    float fail_percentage = (float)failed_estimate_count / (theta_slice_count * phi_slice_count);
    printf("Sector count with failed estimation: %d (%.3f%%)\n", failed_estimate_count, fail_percentage * 100.f);
    printf("%s\n\n", fail_percentage < fail_threshold ? "PASSED" : "FAILED");
}

void test_uniform_hemisphere_sampling() {
    // the number of poins to sample on a sphere
    const int n = 25'000'000;

    // max allowed difference in % between estimated number of samples on the sphere sector and the actual number
    const float error_tolerance_percentage = 0.15f;

    // max percentage of sphere sectors for which estimates is not within error_tolerance_percentage
    const float fail_threshold = 0.01f; 

    const int theta_sector_angle_degrees = 2;
    const int phi_sector_angle_degrees = 4;
    const float theta_sector_angle = radians((float)theta_sector_angle_degrees);
    const float phi_sector_angle = radians((float)phi_sector_angle_degrees);
    const int theta_slice_count = 90 / theta_sector_angle_degrees;
    const int phi_slice_count = 360 / phi_sector_angle_degrees;

    printf("Testing uniform hemisphere sampling...\n");
    printf("N_Hemisphere_Samples = %d, N_Hemisphere_Sectors = %d\n", n, theta_slice_count * phi_slice_count);
    printf("Error tolerance (actual vs estimated sample count): %.1f%%\n", error_tolerance_percentage * 100.f);

    std::vector<std::vector<int>> sphere_sectors;
    sphere_sectors.resize(theta_slice_count);
    for (auto& theta_slice : sphere_sectors)
        theta_slice.resize(phi_slice_count, 0);

    RNG rng;
    rng.init(0, 0x12345);

    Timestamp t;
    int failed_estimate_count = 0;
    for (int i = 0; i < n; i++) {
        Vector2 u = rng.get_vector2();
        Vector3 p = sample_hemisphere_uniform(u);

        ASSERT(p.z >= 0 && p.z <= 1.f);
        float theta = std::acos(p.z);
        theta = std::clamp(theta, 0.f, 0.5f * Pi);
        int theta_slice = std::clamp(int((theta / (0.5f * Pi)) * theta_slice_count), 0, theta_slice_count - 1);

        float phi = std::atan2(p.y, p.x);
        if (phi < 0)
            phi += Pi2;
        phi = std::clamp(phi, 0.f, Pi2);
        int phi_slice = std::clamp(int((phi / Pi2) * phi_slice_count), 0, phi_slice_count - 1);

        sphere_sectors[theta_slice][phi_slice]++;
    }

    for (int theta_slice = 0; theta_slice < theta_slice_count; theta_slice++) {
        for (int phi_slice = 0; phi_slice < phi_slice_count; phi_slice++) {
            float theta_start = radians(float(theta_slice * theta_sector_angle_degrees));
            float phi_start = radians(float(phi_slice * phi_sector_angle_degrees));

            // Area = (phi2 - phi1) * (cos(theta1) - cos(theta2))
            float area = phi_sector_angle * (std::cos(theta_start) - std::cos(theta_start + theta_sector_angle));
            float estimated_sample_count = area / (2 * Pi) * n;

            float sample_count = (float)sphere_sectors[theta_slice][phi_slice];

            float error = std::abs(estimated_sample_count - sample_count) / estimated_sample_count;

            if (error > error_tolerance_percentage)
                failed_estimate_count++;
        }
    }
    float fail_percentage = (float)failed_estimate_count / (theta_slice_count * phi_slice_count);
    printf("Sector count with failed estimation: %d (%.3f%%)\n", failed_estimate_count, fail_percentage * 100.f);
    printf("%s\n\n", fail_percentage < fail_threshold ? "PASSED" : "FAILED");
}

void test_cosine_hemisphere_sampling() {
    // the number of poins to sample on a sphere
    const int n = 50'000'000;

    // max allowed difference in % between estimated number of samples on the sphere sector and the actual number
    const float error_tolerance_percentage = 0.15f;

    // max percentage of sphere sectors for which estimates is not within error_tolerance_percentage
    const float fail_threshold = 0.01f; 

    const int theta_sector_angle_degrees = 1;
    const int phi_sector_angle_degrees = 1;
    const float theta_sector_angle = radians((float)theta_sector_angle_degrees);
    const float phi_sector_angle = radians((float)phi_sector_angle_degrees);
    const int theta_slice_count = 90 / theta_sector_angle_degrees;
    const int phi_slice_count = 360 / phi_sector_angle_degrees;

    printf("Testing cosine hemisphere sampling...\n");
    printf("N_Hemisphere_Samples = %d, N_Hemisphere_Sectors = %d\n", n, theta_slice_count * phi_slice_count);
    printf("Error tolerance (actual vs estimated sample count): %.1f%%\n", error_tolerance_percentage * 100.f);

    std::vector<std::vector<int>> sphere_sectors;
    sphere_sectors.resize(theta_slice_count);
    for (auto& theta_slice : sphere_sectors)
        theta_slice.resize(phi_slice_count, 0);

    RNG rng;
    rng.init(0, 0x12345);

    Timestamp t;
    int failed_estimate_count = 0;
    for (int i = 0; i < n; i++) {
        Vector2 u = rng.get_vector2();
        Vector3 p = sample_hemisphere_cosine(u);

        ASSERT(p.z >= 0 && p.z <= 1.f);
        float theta = std::acos(p.z);
        theta = std::clamp(theta, 0.f, 0.5f * Pi);
        int theta_slice = std::clamp(int((theta / (0.5f * Pi)) * theta_slice_count), 0, theta_slice_count - 1);

        float phi = std::atan2(p.y, p.x);
        if (phi < 0)
            phi += Pi2;
        phi = std::clamp(phi, 0.f, Pi2);
        int phi_slice = std::clamp(int((phi / Pi2) * phi_slice_count), 0, phi_slice_count - 1);

        sphere_sectors[theta_slice][phi_slice]++;
    }

    for (int theta_slice = 0; theta_slice < theta_slice_count; theta_slice++) {
        for (int phi_slice = 0; phi_slice < phi_slice_count; phi_slice++) {
            float theta_start = radians(float(theta_slice * theta_sector_angle_degrees));
            float phi_start = radians(float(phi_slice * phi_sector_angle_degrees));

            // Area = (phi2 - phi1) * (cos(theta1) - cos(theta2))
            float area = phi_sector_angle * (std::cos(theta_start) - std::cos(theta_start + theta_sector_angle));
            float cosine_factor = std::cos(theta_start + 0.5f * theta_sector_angle) / Pi;
            float estimated_sample_count = cosine_factor * area * n;

            float sample_count = (float)sphere_sectors[theta_slice][phi_slice];

            float error = std::abs(estimated_sample_count - sample_count) / estimated_sample_count;

            if (error > error_tolerance_percentage)
                failed_estimate_count++;
        }
    }
    float fail_percentage = (float)failed_estimate_count / (theta_slice_count * phi_slice_count);
    printf("Sector count with failed estimation: %d (%.3f%%)\n", failed_estimate_count, fail_percentage * 100.f);
    printf("%s\n\n", fail_percentage < fail_threshold ? "PASSED" : "FAILED");
}

void test_uniform_cdf_sampling() {
    printf("Testing uniform CDF sampling...\n");
    const int N = 10;
    const float interval_length = 1.f / N;

    std::vector<float> pdf(N, 1.f);
    std::vector<float> cdf(N);
    for (int i = 0; i < N - 1; i++) {
        cdf[i] = (i == 0 ? 0.f : cdf[i-1]) + pdf[i] * interval_length;
    }
    cdf[N-1] = 1.f;

    RNG rng;
    rng.init(0, 0x12345);

    std::vector<int> buckets(100, 0);
    const int Sample_Count = 100000;

    for (int i = 0; i < Sample_Count; i++) {
        float temp_pdf;
        int temp_index;
        float s = sample_from_CDF(rng.get_float(), cdf.data(), int(cdf.size()), interval_length, &temp_pdf, &temp_index);
        ASSERT(s >= 0.f && s < 1.f);
        int bucket_index = int(s * buckets.size());
        ASSERT(bucket_index < (int)buckets.size());
        buckets[bucket_index]++;
    }

    const float error_tolerance = 0.1f;
    const int bucket_max_deviation = int(error_tolerance * Sample_Count / buckets.size());
    const int bucket_estimate = int(Sample_Count / buckets.size());
    int failures = 0;
    for (int n : buckets) {
        if (std::abs(bucket_estimate - n) > bucket_max_deviation)
            failures++;
    }

    const float fail_threshold = 0.02f; 

    printf("Failure count: %d\n", failures);
    printf("%s\n\n", failures <= int(buckets.size() * fail_threshold) ? "PASSED" : "FAILED");
}

void test_non_uniform_cdf_sampling() {
    printf("Testing non-uniform CDF sampling...\n");
    const int N = 10;
    const float interval_length = 1.f / N;

    std::vector<float> pdf(N);
    float sum = 0.f;
    for (int i = 0; i < N; i++) {
        pdf[i] = float(i);
        sum += float(i);
    }
    for (float& p : pdf) {
        p *= N / sum;
    }

    std::vector<float> cdf(N);
    for (int i = 0; i < N - 1; i++) {
        cdf[i] = (i == 0 ? 0.f : cdf[i-1]) + pdf[i] * interval_length;
    }
    cdf[N-1] = 1.f;

    RNG rng;
    rng.init(0, 0x12345);

    std::vector<int> buckets(100, 0);
    const int Sample_Count = 1'000'000;

    for (int i = 0; i < Sample_Count; i++) {
        float temp_pdf;
        int temp_index;
        float s = sample_from_CDF(rng.get_float(), cdf.data(), int(cdf.size()), interval_length, &temp_pdf, &temp_index);
        ASSERT(s >= 0.f && s < 1.f);
        int bucket_index = int(s * buckets.size());
        ASSERT(bucket_index < (int)buckets.size());
        buckets[bucket_index]++;
    }

    const float error_tolerance = 0.1f;
    int failures = 0;
    for (int i = 0; i < int(buckets.size()); i++) {
        int pdf_interval_index = int(float(i) / buckets.size() * N);
        int bucket_estimate = int((pdf[pdf_interval_index] * (1.f / buckets.size())) * Sample_Count);
        int bucket_max_deviation = int(bucket_estimate * error_tolerance);
        if (std::abs(bucket_estimate - buckets[i]) > bucket_max_deviation)
            failures++;
    }

    const float fail_threshold = 0.02f;

    printf("Failure count: %d\n", failures);
    printf("%s\n\n", failures <= int(buckets.size() * fail_threshold) ? "PASSED" : "FAILED");
}

void test_uniform_2d_distribution_sampling() {
    printf("Testing uniform 2D distribution sampling...\n");
    std::vector<float> values(12, 1.f);
    const int nx = 4;
    const int ny = 3;

    Distribution_2D sampler;
    sampler.initialize(values.data(), nx, ny);

    std::vector<int> buckets(nx * ny, 0);
    const int Sample_Count = 10'000;

    RNG rng;
    rng.init(0, 0x12345);

    for (int i = 0; i < Sample_Count; i++) {
        Vector2 u = rng.get_vector2();
        float temp_pdf;
        Vector2 s = sampler.sample(u, &temp_pdf);

        int x = int(s[0] * nx);
        ASSERT(x < nx);
        int y = int(s[1] * ny);
        ASSERT(y < ny);
        buckets[y*nx + x]++;
    }

    const float error_tolerance = 0.1f;
    int bucket_estimate = int(Sample_Count / buckets.size());
    int bucket_max_deviation = int(bucket_estimate * error_tolerance);

    int failures = 0;
    for (int n : buckets) {
        if (std::abs(bucket_estimate - n) > bucket_max_deviation)
            failures++;
    }   
    const float fail_threshold = 0.02f; 
    printf("Failure count: %d\n", failures);
    printf("%s\n\n", failures <= int(buckets.size() * fail_threshold) ? "PASSED" : "FAILED");
}

void test_non_uniform_2d_distribution_sampling() {
    printf("Testing non-uniform 2D distribution sampling...\n");
    const int nx = 4;
    const int ny = 6;
    std::vector<float> values(nx * ny);

    float sum = 0.f;
    for (int i = 0; i < nx*ny; i++) {
        values[i] = float(i);
        sum += float(i);
    }

    Distribution_2D sampler;
    sampler.initialize(values.data(), nx, ny);

    std::vector<int> buckets(nx * ny, 0);
    const int Sample_Count = 100'000;

    RNG rng;
    rng.init(0, 0x12345);

    for (int i = 0; i < Sample_Count; i++) {
        Vector2 u = rng.get_vector2();
        float temp_pdf;
        Vector2 s = sampler.sample(u, &temp_pdf);

        int x = int(s[0] * nx);
        ASSERT(x < nx);
        int y = int(s[1] * ny);
        ASSERT(y < ny);
        buckets[y*nx + x]++;
    }

    const float error_tolerance = 0.1f;
    int failures = 0;
    for (int i = 0; i < int(buckets.size()); i++) {

        int bucket_estimate = int(values[i] / sum * Sample_Count);
        int bucket_max_deviation = int(bucket_estimate * error_tolerance);
        if (std::abs(bucket_estimate - buckets[i]) > bucket_max_deviation)
            failures++;
    }

    const float fail_threshold = 0.02f;

    printf("Failure count: %d\n", failures);
    printf("%s\n\n", failures <= int(buckets.size() * fail_threshold) ? "PASSED" : "FAILED");
}

void test_sampling() {
    test_uniform_sphere_sampling();
    test_uniform_hemisphere_sampling();
    test_cosine_hemisphere_sampling();
    test_uniform_cdf_sampling();
    test_non_uniform_cdf_sampling();
    test_uniform_2d_distribution_sampling();
    test_non_uniform_2d_distribution_sampling();
}
