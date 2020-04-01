#include "std.h"
#include "camera.h"
#include "lib/common.h"

Camera::Camera(const Matrix3x4 & camera_to_world, Vector2 image_extent, float fovy)
    : camera_to_world(camera_to_world)
    , image_extent(image_extent)
{
    float tan_fovy_over_2 = std::tan(radians(fovy/2.f));

    vert_half_dist = tan_fovy_over_2;
    horz_half_dist = (image_extent.x / image_extent.y) * tan_fovy_over_2;
}

Ray Camera::generate_ray(Vector2 film_position) const {
    float u = 2.f * (film_position.x / image_extent.x) - 1.f;
    float v = 2.f * (film_position.y / image_extent.y) - 1.f;

    float dir_x = u * horz_half_dist;
    float dir_z = -v * vert_half_dist;
    Vector3 direction = Vector3(dir_x, 1.f, dir_z).normalized();

    Ray camera_ray = Ray(Vector3_Zero, direction);
    Ray world_ray = transform_ray(camera_to_world, camera_ray);
    return world_ray;
}
