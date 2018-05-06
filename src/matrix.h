#pragma once

#include "ray.h"

struct Matrix4x4 {
    float a[4][4];
};

struct Matrix3x4 {
    float a[3][4];
};

Matrix3x4 operator*(const Matrix3x4& m1, const Matrix3x4& m2);

Vector transform_point(const Matrix3x4& m, Vector p);
Vector transform_vector(const Matrix3x4& m, Vector v);
Ray transform_ray(const Matrix3x4& m, const Ray& ray);
