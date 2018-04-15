#pragma once

#include <cmath>

class Vector
{
public:
    Vector() {}

    explicit Vector(float v)
        : x(v), y(v), z(v) {}

    Vector(float x, float y, float z)
        : x(x), y(y), z(z) {}

    Vector operator-() const {
        return Vector(-x, -y, -z);
    }

    float operator[](int index) const {
        return (&x)[index];
    }

    float& operator[](int index) {
        return (&x)[index];
    }

    Vector& operator+=(const Vector& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector& operator-=(const Vector& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector& operator*=(const Vector& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    Vector& operator*=(float t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }

    Vector& operator/=(float t) {
        x /= t;
        y /= t;
        z /= t;
        return *this;
    }

    Vector operator/(float t) const {
        return Vector(x/t, y/t, z/t);
    }

    float length() const {
        return std::sqrt(squared_length());
    }

    float squared_length() const {
        return x*x + y*y + z*z;
    }

    Vector normalized() const {
        return *this / length();
    }

    float x, y, z;
};

inline Vector operator+(const Vector& v1, const Vector& v2) {
    return Vector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

inline Vector operator-(const Vector& v1, const Vector& v2) {
    return Vector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

inline Vector operator*(const Vector& v1, const Vector& v2) {
    return Vector(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
}

inline Vector operator*(const Vector& v, float t) {
    return Vector(v.x * t, v.y * t, v.z * t);
}

inline Vector operator*(float t, const Vector& v) {
    return v * t;
}

inline float dot_product(const Vector& v1, const Vector& v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

inline Vector cross_product(const Vector& v1, const Vector& v2) {
    return Vector(
        v1.y*v2.z - v1.z*v2.y,
        v1.z*v2.x - v1.x*v2.z,
        v1.x*v2.y - v1.y*v2.x);
}
