#include "ray.h"
#include "intersection.h"
#include "triangle_mesh.h"

//
// Möller-Trumbore triangle intersection algorithm.
// http://www.graphics.cornell.edu/pubs/1997/MT97.pdf
//
float intersect_triangle_moller_trumbore(const Ray& ray, const Vector& p0, const Vector& p1, const Vector& p2, float& b1, float& b2) {
    Vector edge1 = p1 - p0;
    Vector edge2 = p2 - p0;

    Vector p = cross(ray.d, edge2);
    float divisor = dot(edge1, p);

    // todo: do we need to check against epsilon for better numeric stability?
    if (divisor == 0.0)
        return Infinity;

    const float inv_divisor = 1.0f / divisor;

    // compute barycentric coordinate b1
    Vector t = ray.o - p0;
    b1 = inv_divisor * dot(t, p);
    if (b1 < 0.0 || b1 > 1.0)
        return Infinity;

    // compute barycentric coordnate b2
    Vector q = cross(t, edge1);
    b2 = inv_divisor * dot(ray.d, q);
    if (b2 < 0.0 || b1 + b2 > 1.0)
        return Infinity;

    // compute distance from ray origin to intersection point
    float distance = inv_divisor * dot(edge2, q);
    if (distance < 0.0)
        return Infinity;

    return distance;
}

Triangle_Intersection intersect_triangle(const Ray& ray, const Triangle_Mesh* mesh, int32_t triangle_index) {
    Vector p0, p1, p2;
    mesh->get_triangle(triangle_index, p0, p1, p2);

    Triangle_Intersection isect;
    isect.t = intersect_triangle_moller_trumbore(ray, p0, p1,p2, isect.b1, isect.b2);
    isect.mesh = mesh;
    isect.triangle_index = triangle_index;
    return isect;
}

Local_Geometry::Local_Geometry(const Ray& ray, const Triangle_Intersection& triangle_intersection) {
    Vector p0, p1, p2;
    triangle_intersection.mesh->get_triangle(triangle_intersection.triangle_index, p0, p1, p2);

    position = ray.get_point(triangle_intersection.t);
    normal = cross(p1 - p0, p2  - p0).normalized();
}
