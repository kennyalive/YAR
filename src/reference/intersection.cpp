#include "std.h"
#include "lib/common.h"
#include "intersection.h"

#include "lib/ray.h"
#include "lib/triangle_mesh.h"

//
// Möller-Trumbore triangle intersection algorithm.
// https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
//
float intersect_triangle_möller_trumbore(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, Vector3* barycentrics)
{
    Vector3 edge1 = p1 - p0;
    Vector3 edge2 = p2 - p0;

    Vector3 p = cross(ray.direction, edge2);
    float divisor = dot(edge1, p);

    if (divisor == 0.0)
        return Infinity;

    const float inv_divisor = 1.0f / divisor;
    Vector3 b;

    // compute barycentric coordinate b1
    Vector3 t = ray.origin - p0;
    b[1] = inv_divisor * dot(t, p);
    if (b[1] < 0.0 || b[1] > 1.0)
        return Infinity;

    // compute barycentric coordnate b2
    Vector3 q = cross(t, edge1);
    b[2] = inv_divisor * dot(ray.direction, q);
    if (b[2] < 0.0 || b[1] + b[2] > 1.0)
        return Infinity;

    // compute distance from ray origin to intersection point
    float distance = inv_divisor * dot(edge2, q);
    if (distance < 0.0)
        return Infinity;

    b[0] = 1.f - (b[1] + b[2]);
    *barycentrics = b;
    return distance;
}

//
// Sven Woop, Carsten Benthin, and Ingo Wald, Watertight Ray/Triangle Intersection, 
// Journal of Computer Graphics Techniques (JCGT), vol. 2, no. 1, 65-82, 2013
// http://jcgt.org/published/0002/01/05/
//
float intersect_triangle_watertight(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, Vector3* barycentrics)
{
    const int kz = ray.direction.abs().max_dimension();
    const int kx = (kz == 2 ? 0 : kz + 1);
    const int ky = (kz == 0 ? 2 : kz - 1);

    Vector3 direction = ray.direction.permutation(kx, ky, kz);
    float sx = -direction.x / direction.z;
    float sy = -direction.y / direction.z;
    float sz = 1.f / direction.z;

    const Vector3 p0t = (p0 - ray.origin).permutation(kx, ky, kz);
    const Vector3 p1t = (p1 - ray.origin).permutation(kx, ky, kz);
    const Vector3 p2t = (p2 - ray.origin).permutation(kx, ky, kz);

    float x0 = p0t.x + sx * p0t.z;
    float y0 = p0t.y + sy * p0t.z;
    float x1 = p1t.x + sx * p1t.z;
    float y1 = p1t.y + sy * p1t.z;
    float x2 = p2t.x + sx * p2t.z;
    float y2 = p2t.y + sy * p2t.z;

    float e0 = x1*y2 - y1*x2;
    float e1 = x2*y0 - y2*x0;
    float e2 = x0*y1 - y0*x1;

    if (e0 == 0.f || e1 == 0.f || e2 == 0.f) {
        e0 = float(double(x1)*double(y2) - double(y1)*double(x2));
        e1 = float(double(x2)*double(y0) - double(y2)*double(x0));
        e2 = float(double(x0)*double(y1) - double(y0)*double(x1));
    }

    if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
        return Infinity;

    float det = e0 + e1 + e2;
    if (det == 0.f)
        return Infinity;

    float z0 = sz * p0t.z;
    float z1 = sz * p1t.z;
    float z2 = sz * p2t.z;

    float t_scaled = e0*z0 + e1*z1 + e2*z2;

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
    float t = inv_det * t_scaled;
    ASSERT(t >= 0);

    barycentrics->x = e0 * inv_det;
    barycentrics->y = e1 * inv_det;
    barycentrics->z = e2 * inv_det;
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

        Vector3 b;
        float t = intersect_triangle_watertight(ray, p0, p1, p2, &b);

        if (t < intersection.t) {
            intersection.t = t;
            intersection.geometry_type = geometry.type;
            intersection.triangle_intersection.barycentrics = b;
            intersection.triangle_intersection.mesh = mesh;
            intersection.triangle_intersection.triangle_index = primitive_index;
        }
    }
    else {
        ASSERT(false);
    }
}
