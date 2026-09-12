#include "std.h"
#include "common.h"
#include "light.h"

bool Lights::has_lights() const {
    return
        point_lights.size() ||
        spot_lights.size() ||
        directional_lights.size() ||
        diffuse_rectangular_lights.size() ||
        diffuse_sphere_lights.size() ||
        diffuse_triangle_mesh_lights.size() ||
        has_environment_light;
}

void Lights::update_total_light_count() {
    total_light_count =
        (int)point_lights.size() + 
        (int)spot_lights.size() +
        (int)directional_lights.size() +
        (int)diffuse_rectangular_lights.size() +
        (int)diffuse_sphere_lights.size() +
        (int)diffuse_triangle_mesh_lights.size() +
        has_environment_light;
}
