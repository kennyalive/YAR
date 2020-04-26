#pragma once

#include "lib/matrix.h"
#include "lib/ray.h"

/*
Camera space:

Z is up:
            ^ Z
            |   ^ Y - camera direction is Y axis
            |  /
            | /
      ------------> X
            |
            |


Y is up:
           ^ Y
            |  / - camera direction is negative Z
            | /
            |/
     ------------> X
           /|
          / |
         v 
         Z
*/

/*
Image space: film position(0, 0) corresponds to the upper left corner.
*/

class Camera {
public:
    Camera(const Matrix3x4& camera_to_world, Vector2 image_extent, float fovy, bool z_is_up);
    Ray generate_ray(Vector2 film_position) const;

private:
    Matrix3x4   camera_to_world;
    Vector2     image_extent;
    float       horz_half_dist;
    float       vert_half_dist;
    bool        z_is_up;
};
