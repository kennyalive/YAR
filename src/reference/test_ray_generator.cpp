#include "std.h"
#include "sampling.h"
#include "test_ray_generator.h"
#include "lib/common.h"
#include "lib/vector.h"

Ray_Generator::Ray_Generator(const Bounding_Box& mesh_bounds) {
    pcg32_srandom_r(&rng, 0, 0);

    auto diagonal = mesh_bounds.max_p - mesh_bounds.min_p;
    float delta = 2.0f * diagonal.length();

    auto p_min = mesh_bounds.min_p - Vector3(delta);
    auto p_max = mesh_bounds.max_p + Vector3(delta);
    ray_bounds = Bounding_Box(p_min, p_max);
}

Ray Ray_Generator::generate_ray(const Vector3& last_hit, float last_hit_epsilon) {
    auto random_from_range = [this](float a, float b) {
        return a + (b - a) * random_float(&rng);
    };

    // Ray origin.
    Vector3 origin;
    origin.x = random_from_range(ray_bounds.min_p.x, ray_bounds.max_p.x);
    origin.y = random_from_range(ray_bounds.min_p.y, ray_bounds.max_p.y);
    origin.z = random_from_range(ray_bounds.min_p.z, ray_bounds.max_p.z);

    const bool use_last_hit = random_float(&rng) < 0.25f;
    if (use_last_hit)
        origin = last_hit;

    // Ray direction.
    auto direction = uniform_sample_sphere(random_float(&rng), random_float(&rng));
    auto len = direction.length();

    if (random_float(&rng) < 1.0f / 32.0f && direction.z != 0.0)
        direction.x = direction.y = 0.0;
    else if (random_float(&rng) < 1.0f / 32.0f && direction.y != 0.0)
        direction.x = direction.z = 0.0;
    else if (random_float(&rng) < 1.0f / 32.0f && direction.x != 0.0)
        direction.y = direction.z = 0.0;
    direction = direction.normalized();

    auto ray = Ray(origin, direction);
    ray.origin = ray.get_point(use_last_hit ? last_hit_epsilon : 1e-3f);
    return ray;
}
