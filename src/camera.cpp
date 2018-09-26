#include "common.h"
#include "camera.h"

Camera::Camera(const Matrix3x4 & camera_to_world, Vector2 image_extent, float fovy)
    : camera_to_world(camera_to_world)
    , image_extent(image_extent)
{
    float tan_fovy_over_2 = std::tan(radians(fovy/2.f));

    // Here we assume that distance to virtual film plane is 1.0 along Z axis in camera space.
    horz_half_dist = (image_extent.x / image_extent.y) * tan_fovy_over_2;
    vert_half_dist = tan_fovy_over_2;
}

Ray Camera::generate_ray(Vector2 film_position) const {
    assert(film_position.x >= 0.f && film_position.x <= image_extent.x);
    assert(film_position.y >= 0.f && film_position.y <= image_extent.y);

    // map film_position to [-1, 1] range
    float u = 2.f * (film_position.x / image_extent.x) - 1.f;
    float v = 2.f * (film_position.y / image_extent.y) - 1.f;

    float dir_x = u * horz_half_dist;
    float dir_y = v * vert_half_dist;

    Vector dir = Vector(dir_x, dir_y, 1.f).normalized();

    Ray camera_ray = Ray(Vector(0), dir);
    Ray world_ray = transform_ray(camera_to_world, camera_ray);
    return world_ray;
}
