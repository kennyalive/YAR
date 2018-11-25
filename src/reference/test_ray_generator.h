#pragma once

#include "lib/bounding_box.h"
#include "lib/rng.h"

struct Vector3;

class Ray_Generator {
public:
    Ray_Generator(const Bounding_Box& mesh_bounds);
    Ray generate_ray(const Vector3& last_hit, float last_hit_epsilon);

private:
    RNG rng;
    Bounding_Box ray_bounds;
};
