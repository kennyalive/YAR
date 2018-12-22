#include "intersection.h"
#include "triangle_mesh.h"
#include "lib/ray.h"

//
// Möller-Trumbore triangle intersection algorithm.
// http://www.graphics.cornell.edu/pubs/1997/MT97.pdf
//
float intersect_triangle_moller_trumbore(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, float& b1, float& b2) {
    Vector3 edge1 = p1 - p0;
    Vector3 edge2 = p2 - p0;

    Vector3 p = cross(ray.direction, edge2);
    float divisor = dot(edge1, p);

    // todo: do we need to check against epsilon for better numeric stability?
    if (divisor == 0.0)
        return Infinity;

    const float inv_divisor = 1.0f / divisor;

    // compute barycentric coordinate b1
    Vector3 t = ray.origin - p0;
    b1 = inv_divisor * dot(t, p);
    if (b1 < 0.0 || b1 > 1.0)
        return Infinity;

    // compute barycentric coordnate b2
    Vector3 q = cross(t, edge1);
    b2 = inv_divisor * dot(ray.direction, q);
    if (b2 < 0.0 || b1 + b2 > 1.0)
        return Infinity;

    // compute distance from ray origin to intersection point
    float distance = inv_divisor * dot(edge2, q);
    if (distance < 0.0)
        return Infinity;

    return distance;
}

void intersect_triangle(const Ray& ray, const Triangle_Mesh* mesh, int32_t triangle_index, Triangle_Intersection& intersection) {
    Vector3 p0, p1, p2;
    mesh->get_triangle(triangle_index, p0, p1, p2);

    float b1, b2;
    float t = intersect_triangle_moller_trumbore(ray, p0, p1, p2, b1, b2);

    if (t < intersection.t) {
        intersection.t = t;
        intersection.b1 = b1;
        intersection.b2 = b2;
        intersection.mesh = mesh;
        intersection.triangle_index = triangle_index;
    }
}

Local_Geometry::Local_Geometry(const Ray& ray, const Triangle_Intersection& triangle_intersection) {
    Vector3 p0, p1, p2;
    triangle_intersection.mesh->get_triangle(triangle_intersection.triangle_index, p0, p1, p2);

    position = ray.get_point(triangle_intersection.t);
    normal = cross(p1 - p0, p2  - p0).normalized();
    k_diffuse = triangle_intersection.mesh->k_diffuse;
    k_specular = triangle_intersection.mesh->k_specular;
}
