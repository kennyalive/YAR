#include "std.h"
#include "lib/common.h"

#include "lib/random.h"

void test_random_uint32_distribution() {
    const int n = 100'000'000;

    const uint32_t bucket_size = 1024 * 256;
    const uint32_t bucket_count = (uint64_t(1) << 32) / bucket_size;

    const float error_tolerance_percentage = 0.05f;
    const float fail_threshold = 0.01f;

    printf("Testing random uint32 distribution...\n");
    printf("Bucket count = %d\n", bucket_count);

    std::vector<int> buckets(bucket_count, 0);
    RNG rng;
    rng.init(0, 0);
    for (int i = 0; i < n; i++) {
        uint32_t k = rng.get_uint();
        int bucket_index = k / bucket_size;
        ASSERT(bucket_index < bucket_count);
        buckets[bucket_index]++;
    }

    const float estimated_bucket_item_count = float(n) / bucket_count;
    int failed_estimate_count = 0;
    for (int i = 0; i < bucket_count; i++) {
        float error = std::abs(buckets[i] - estimated_bucket_item_count) /  estimated_bucket_item_count;
        if (error > error_tolerance_percentage)
            failed_estimate_count++;
    }
    float fail_percentage = (float)failed_estimate_count / bucket_count;
    printf("Bucket count with failed estimation: %d (%.3f%%)\n", failed_estimate_count, fail_percentage * 100.f);
    printf("%s\n\n", fail_percentage < fail_threshold ? "PASSED" : "FAILED");
}

void test_random_uint32_distribution_multiple_streams() {
    const int n = 50'000'000;
    const int stream_count = 64;

    const uint32_t bucket_size = 1024 * 512;
    const uint32_t bucket_count = (uint64_t(1) << 32) / bucket_size;

    const float error_tolerance_percentage = 0.05f;
    const float fail_threshold = 0.01f;

    printf("Testing random uint32 distribution using multiple streams...\n");
    printf("Bucket count = %d, stream count = %d\n", bucket_count, stream_count);

    int failed_estimate_count = 0;
    const float estimated_bucket_item_count = float(n) / bucket_count;
    for (int stream = 0; stream < stream_count; stream++) {
        std::vector<int> buckets(bucket_count, 0);
        RNG rng;
        rng.init(0, stream);
        for (int i = 0; i < n; i++) {
            uint32_t k = rng.get_uint();
            int bucket_index = k / bucket_size;
            ASSERT(bucket_index < bucket_count);
            buckets[bucket_index]++;
        }

        for (int i = 0; i < bucket_count; i++) {
            float error = std::abs(buckets[i] - estimated_bucket_item_count) /  estimated_bucket_item_count;
            if (error > error_tolerance_percentage)
                failed_estimate_count++;
        }
    }
    float fail_percentage = (float)failed_estimate_count / (bucket_count * stream_count);
    printf("Bucket count with failed estimation: %d (%.3f%%)\n", failed_estimate_count, fail_percentage * 100.f);
    printf("%s\n\n", fail_percentage < fail_threshold ? "PASSED" : "FAILED");
}

void test_random_bounded_uint32_distribution() {
    const float error_tolerance_percentage = 0.05f;
    const float fail_threshold = 0.01f;
    const int estimated_bucket_item_count = 4'000;

    printf("Testing random bounded uint32 distribution...\n");

    std::vector<uint32_t> bounds { 1, 2, 4, 5, 7, 8, 16, 24, 39, 64, 100, 256, 1001, 4096, 11111 };
    for (int method = 0; method < 2; method++) {
        if (method == 0)
            printf("Using get_bounded_uint function\n");
        else
            printf("Using get_bounded_uint_fast_and_biased function\n");

        bool failed = false;
        for (uint32_t bound : bounds) {
            printf("Bound value: %u, ", bound);

            std::vector<int> buckets(bound, 0);
            RNG rng;
            rng.init(0, 0);
            const int n = estimated_bucket_item_count * bound;
            for (int i = 0; i < n; i++) {
                uint32_t k;
                if (method == 0)
                    k = rng.get_bounded_uint(bound);
                else
                    k = rng.get_bounded_uint_fast_and_biased(bound);
                ASSERT(k < bound);
                buckets[k]++;
            }

            int failed_estimate_count = 0;
            for (int i = 0; i < (int)bound; i++) {
                float error = std::abs(buckets[i] - estimated_bucket_item_count) / (float)estimated_bucket_item_count;
                if (error > error_tolerance_percentage)
                    failed_estimate_count++;
            }
            float fail_percentage = (float)failed_estimate_count / bound;
            printf("bucket count with failed estimation: %d\n", failed_estimate_count);
            if (fail_percentage >= fail_threshold) {
                failed = true;
                break;
            }
        }
        printf("%s\n\n", !failed ? "PASSED" : "FAILED");
    }
}

void test_random_float() {
    const int n = 500'000'000;
    const uint32_t bucket_count = 100'000;

    const float error_tolerance_percentage = 0.05f;
    const float fail_threshold = 0.01f;

    printf("Testing random float distribution...\n");
    printf("Bucket count = %d\n", bucket_count);

    std::vector<int> buckets(bucket_count, 0);
    RNG rng;
    rng.init(0, 0);
    for (int i = 0; i < n; i++) {
        float f = rng.get_float();
        ASSERT(f >= 0 && f < 1.f);
        int bucket_index = std::min(int(f * bucket_count), int(bucket_count - 1));
        buckets[bucket_index]++;
    }

    const float estimated_bucket_item_count = float(n) / bucket_count;
    int failed_estimate_count = 0;
    for (int i = 0; i < bucket_count; i++) {
        float error = std::abs(buckets[i] - estimated_bucket_item_count) /  estimated_bucket_item_count;
        if (error > error_tolerance_percentage)
            failed_estimate_count++;
    }
    float fail_percentage = (float)failed_estimate_count / bucket_count;
    printf("Bucket count with failed estimation: %d (%.3f%%)\n", failed_estimate_count, fail_percentage * 100.f);
    printf("%s\n\n", fail_percentage < fail_threshold ? "PASSED" : "FAILED");
}

void test_random() {
    test_random_uint32_distribution();
    test_random_uint32_distribution_multiple_streams();
    test_random_bounded_uint32_distribution();
    test_random_float();
}
