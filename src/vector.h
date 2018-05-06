#pragma once

#include <cmath>

struct Vector {
    float x, y, z;

    Vector() {}

    explicit Vector(float v)
        : x(v), y(v), z(v) {}

    Vector(float x, float y, float z)
        : x(x), y(y), z(z) {}

    bool operator==(Vector v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(Vector v) const {
        return !(*this == v);
    }

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
};

struct Vector2 {
    float x, y;

    Vector2() {}

    explicit Vector2(float v)
        : x(v), y(v) {}

    Vector2(float x, float y)
        : x(x), y(y) {}

    float operator[](int index) const {
        return (&x)[index];
    }

    float& operator[](int index) {
        return (&x)[index];
    }
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

inline float dot(const Vector& v1, const Vector& v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

inline Vector cross(const Vector& v1, const Vector& v2) {
    return Vector(
        v1.y*v2.z - v1.z*v2.y,
        v1.z*v2.x - v1.x*v2.z,
        v1.x*v2.y - v1.y*v2.x);
}
