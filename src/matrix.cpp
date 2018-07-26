#include "matrix.h"

Matrix3x4 operator*(const Matrix3x4& m1, const Matrix3x4& m2) {
    Matrix3x4 m;

    m.a[0][0] = m1.a[0][0]*m2.a[0][0] + m1.a[0][1]*m2.a[1][0] + m1.a[0][2]*m2.a[2][0];
    m.a[0][1] = m1.a[0][0]*m2.a[0][1] + m1.a[0][1]*m2.a[1][1] + m1.a[0][2]*m2.a[2][1];
    m.a[0][2] = m1.a[0][0]*m2.a[0][2] + m1.a[0][1]*m2.a[1][2] + m1.a[0][2]*m2.a[2][2];
    m.a[0][3] = m1.a[0][0]*m2.a[0][3] + m1.a[0][1]*m2.a[1][3] + m1.a[0][2]*m2.a[2][3] + m1.a[0][3];

    m.a[1][0] = m1.a[1][0]*m2.a[0][0] + m1.a[1][1]*m2.a[1][0] + m1.a[1][2]*m2.a[2][0];
    m.a[1][1] = m1.a[1][0]*m2.a[0][1] + m1.a[1][1]*m2.a[1][1] + m1.a[1][2]*m2.a[2][1];
    m.a[1][2] = m1.a[1][0]*m2.a[0][2] + m1.a[1][1]*m2.a[1][2] + m1.a[1][2]*m2.a[2][2];
    m.a[1][3] = m1.a[1][0]*m2.a[0][3] + m1.a[1][1]*m2.a[1][3] + m1.a[1][2]*m2.a[2][3] + m1.a[1][3];

    m.a[2][0] = m1.a[2][0]*m2.a[0][0] + m1.a[2][1]*m2.a[1][0] + m1.a[2][2]*m2.a[2][0];
    m.a[2][1] = m1.a[2][0]*m2.a[0][1] + m1.a[2][1]*m2.a[1][1] + m1.a[2][2]*m2.a[2][1];
    m.a[2][2] = m1.a[2][0]*m2.a[0][2] + m1.a[2][1]*m2.a[1][2] + m1.a[2][2]*m2.a[2][2];
    m.a[2][3] = m1.a[2][0]*m2.a[0][3] + m1.a[2][1]*m2.a[1][3] + m1.a[2][2]*m2.a[2][3] + m1.a[2][3];

    return m;
}

Vector transform_point(const Matrix3x4& m, Vector p) {
    Vector p2;
    p2.x = m.a[0][0]*p.x + m.a[0][1]*p.y + m.a[0][2]*p.z + m.a[0][3];
    p2.y = m.a[1][0]*p.x + m.a[1][1]*p.y + m.a[1][2]*p.z + m.a[1][3];
    p2.z = m.a[2][0]*p.x + m.a[2][1]*p.y + m.a[2][2]*p.z + m.a[2][3];
    return p2;
}

Vector transform_vector(const Matrix3x4& m, Vector v) {
    Vector v2;
    v2.x = m.a[0][0]*v.x + m.a[0][1]*v.y + m.a[0][2]*v.z;
    v2.y = m.a[1][0]*v.x + m.a[1][1]*v.y + m.a[1][2]*v.z;
    v2.z = m.a[2][0]*v.x + m.a[2][1]*v.y + m.a[2][2]*v.z;
    return v2;
}

Ray transform_ray(const Matrix3x4& m, const Ray& ray) {
    Ray ray2;
    ray2.origin = transform_point(m, ray.origin);
    ray2.direction = transform_vector(m, ray.direction);
    return ray2;
}
