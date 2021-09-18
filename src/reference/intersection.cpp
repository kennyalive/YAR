#include "std.h"
#include "lib/common.h"
#include "intersection.h"

#include "lib/ray.h"
#include "lib/triangle_mesh.h"

//
// Möller-Trumbore triangle intersection algorithm.
// https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
//
float intersect_triangle_möller_trumbore(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, float& b1, float& b2)
{
    Vector3 edge1 = p1 - p0;
    Vector3 edge2 = p2 - p0;

    Vector3 p = cross(ray.direction, edge2);
    float divisor = dot(edge1, p);

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

//
// Sven Woop, Carsten Benthin, and Ingo Wald, Watertight Ray/Triangle Intersection, 
// Journal of Computer Graphics Techniques (JCGT), vol. 2, no. 1, 65-82, 2013
// http://jcgt.org/published/0002/01/05/
//
float intersect_triangle_watertight(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, float& b1, float& b2)
{
    const int kx_lookup[3] = {1, 2, 0};
    const int ky_lookup[3] = {2, 0, 1};
    const int kz = ray.direction.abs().max_dimension();
    const int kx = kx_lookup[kz];
    const int ky = ky_lookup[kz];

    Vector3 direction = ray.direction.permutation(kx, ky, kz);
    float sx = -direction.x / direction.z;
    float sy = -direction.y / direction.z;
    float sz = 1.f / direction.z;

    const Vector3 a = (p0 - ray.origin).permutation(kx, ky, kz);
    const Vector3 b = (p1 - ray.origin).permutation(kx, ky, kz);
    const Vector3 c = (p2 - ray.origin).permutation(kx, ky, kz);

    float ax = a.x + sx * a.z;
    float ay = a.y + sy * a.z;
    float bx = b.x + sx * b.z;
    float by = b.y + sy * b.z;
    float cx = c.x + sx * c.z;
    float cy = c.y + sy * c.z;

    float u = bx*cy - cx*by;
    float v = cx*ay - ax*cy;
    float w = ax*by - bx*ay;

    if (u == 0.f || v == 0.f || w == 0.f) {
        u = float(double(bx)*double(cy) - double(cx)*double(by));
        v = float(double(cx)*double(ay) - double(ax)*double(cy));
        w = float(double(ax)*double(by) - double(bx)*double(ay));
    }

    if ((u < 0 || v < 0 || w < 0) && (u > 0 || v > 0 || w > 0))
        return Infinity;

    float det = u + v + w;
    if (det == 0.f)
        return Infinity;

    float az = sz * a.z;
    float bz = sz * b.z;
    float cz = sz * c.z;

    float t_scaled = u*az + v*bz + w*cz;

    // This following code is more efficient version of this snippet:
    //    if (det < 0 && t_scaled > 0 || det > 0 && t_scaled < 0) return Infinity;
    // MSVC compiler generates 2 x64 instructions: xor, jl.
    // Benchmark consistenly shows 1 cycle gain :)
    // TODO: replace memcpy with std::bit_cast if no surprises in generated code
    {
        uint32_t det_bits;
        memcpy(&det_bits, &det, 4);
        uint32_t t_scaled_bits;
        memcpy(&t_scaled_bits, &t_scaled, 4);
        if ((det_bits ^ t_scaled_bits) >> 31)
            return Infinity;
    }

    float inv_det = 1.f / det;
    b1 = v * inv_det;
    b2 = w * inv_det;
    float t = inv_det * t_scaled;
    ASSERT(t >= 0 && b1 >= 0 && b2 >= 0);
    return t;
}

void intersect_geometric_primitive(const Ray& ray,
    const Geometries* geometries, Geometry_Handle geometry, int primitive_index,
    Intersection& intersection)
{
    if (geometry.type == Geometry_Type::triangle_mesh) {
        const Triangle_Mesh* mesh = &geometries->triangle_meshes[geometry.index];
        Vector3 p0, p1, p2;
        mesh->get_triangle(primitive_index, p0, p1, p2);

        float b1, b2;
        float t = intersect_triangle_watertight(ray, p0, p1, p2, b1, b2);

        if (t < intersection.t) {
            intersection.t = t;
            intersection.geometry_type = geometry.type;
            intersection.triangle_intersection.b1 = b1;
            intersection.triangle_intersection.b2 = b2;
            intersection.triangle_intersection.mesh = mesh;
            intersection.triangle_intersection.triangle_index = primitive_index;
        }
    }
    else {
        ASSERT(false);
    }
}
