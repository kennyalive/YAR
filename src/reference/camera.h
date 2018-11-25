#pragma once

#include "lib/matrix.h"
#include "lib/ray.h"

/*
Camera space:

            |   ^  Z (points into the scene)
            |  /
            | /
       -----------> X
            |
            |
            v  Y
*/

class Camera {
public:
    Camera(const Matrix3x4& camera_to_world, Vector2 image_extent, float fovy);
    Ray generate_ray(Vector2 film_position) const;

private:
    Matrix3x4   camera_to_world;
    Vector2     image_extent;
    float       horz_half_dist;
    float       vert_half_dist;
};
