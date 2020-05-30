#pragma once

#include "common.h"

#include <cmath>

struct Vector2;
struct Vector2i;
struct Vector4;

//
// Vector3
//
struct Vector3 {
    float x, y, z;

    Vector3()
        : x(0.f), y(0.f), z(0.f) {}

    constexpr explicit Vector3(float v)
        : x(v), y(v), z(v) {}

    explicit Vector3(const Vector4& v);

    explicit Vector3(const float* v)
        : x(v[0]), y(v[1]), z(v[2]) {}

    Vector3(float x, float y, float z)
        : x(x), y(y), z(z) {}

    bool operator==(Vector3 v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(Vector3 v) const {
        return !(*this == v);
    }

    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    float operator[](int index) const {
        return (&x)[index];
    }

    float& operator[](int index) {
        return (&x)[index];
    }

    Vector3& operator+=(const Vector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3& operator*=(const Vector3& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }

    Vector3& operator*=(float t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }

    Vector3& operator/=(float t) {
        x /= t;
        y /= t;
        z /= t;
        return *this;
    }

    Vector3 operator/(float t) const {
        return Vector3(x/t, y/t, z/t);
    }

    float length() const {
        return std::sqrt(length_squared());
    }

    float length_squared() const {
        return x*x + y*y + z*z;
    }

    Vector3 normalized() const {
        return *this / length();
    }

    void normalize() {
        *this /= length();
    }

    bool is_normalized(float epsilon = 1e-3f) const {
        return std::abs(length() - 1.f) < epsilon;
    }

    Vector3 abs() const {
        return Vector3(std::abs(x), std::abs(y), std::abs(z));
    }
};

constexpr Vector3 Vector3_Zero = Vector3(0.f);

inline Vector3 operator+(const Vector3& v1, const Vector3& v2) {
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

inline Vector3 operator-(const Vector3& v1, const Vector3& v2) {
    return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

inline Vector3 operator*(const Vector3& v1, const Vector3& v2) {
    return Vector3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
}

inline Vector3 operator*(const Vector3& v, float t) {
    return Vector3(v.x * t, v.y * t, v.z * t);
}

inline Vector3 operator*(float t, const Vector3& v) {
    return v * t;
}

inline float dot(const Vector3& v1, const Vector3& v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

inline Vector3 cross(const Vector3& v1, const Vector3& v2) {
    return Vector3(
        v1.y*v2.z - v1.z*v2.y,
        v1.z*v2.x - v1.x*v2.z,
        v1.x*v2.y - v1.y*v2.x);
}

namespace std {
template<> struct hash<Vector3> {
    size_t operator()(Vector3 v) const {
        size_t hash = 0;
        hash_combine(hash, v.x);
        hash_combine(hash, v.y);
        hash_combine(hash, v.z);
        return hash;
    }
};
}

//
// Vector2
//
struct Vector2 {
    union {
        struct { float x, y; };
        struct { float u, v; };
    };

    Vector2()
        : x(0.f), y(0.f) {}

    constexpr explicit Vector2(float v)
        : x(v), y(v) {}

    explicit Vector2(const float* v)
        : x(v[0]), y(v[1]) {}

    Vector2(float x, float y)
        : x(x), y(y) {}

    explicit Vector2(const Vector2i& v);

    bool operator==(Vector2 v) const {
        return x == v.x && y == v.y;
    }

    bool operator!=(Vector2 v) const {
        return !(*this == v);
    }

    float operator[](int index) const {
        return (&x)[index];
    }

    float& operator[](int index) {
        return (&x)[index];
    }

    Vector2& operator*=(float t) {
        x *= t;
        y *= t;
        return *this;
    }

    float length() const {
        return std::sqrt(length_squared());
    }

    float length_squared() const {
        return x*x + y*y;
    }
};

constexpr Vector2 Vector2_Zero = Vector2(0.f);

inline Vector2 operator+(Vector2 a, Vector2 b) {
    return Vector2{a.x + b.x, a.y + b.y};
}

inline Vector2 operator-(Vector2 a, Vector2 b) {
    return Vector2{a.x - b.x, a.y - b.y};
}

inline Vector2 operator*(const Vector2& v, float t) {
    return Vector2(v.x * t, v.y * t);
}

inline Vector2 operator*(float t, const Vector2& v) {
    return v * t;
}

inline Vector2 operator*(const Vector2& v1, const Vector2& v2) {
    return Vector2(v1.x * v2.x, v1.y * v2.y);
}

inline bool operator<(Vector2 v1, Vector2 v2) {
    return v1[0] < v2[0] && v1[1] < v2[1];
}

inline bool operator<=(Vector2 v1, Vector2 v2) {
    return v1[0] <= v2[0] && v1[1] <= v2[1];
}

inline bool operator>(Vector2 v1, Vector2 v2) {
    return v1[0] > v2[0] && v1[0] > v2[0];
}

inline bool operator>=(Vector2 v1, Vector2 v2) {
    return v1[0] >= v2[0] && v1[0] >= v2[0];
}

namespace std {
template<> struct hash<Vector2> {
    size_t operator()(Vector2 v) const {
        size_t hash = 0;
        hash_combine(hash, v.x);
        hash_combine(hash, v.y);
        return hash;
    }
};
}

//
// Vector4
//
struct Vector4 {
    float x, y, z, w;

    Vector4()
        : x(0.f), y(0.f), z(0.f), w(0.f) {}

    constexpr explicit Vector4(float v)
        : x(v), y(v), z(v), w(v) {}

    Vector4(float x, float y, float z, float w)
        : x(x), y(y), z(z), w(w) {}

    Vector4(Vector3 xyz, float w)
        : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}

    bool operator==(Vector4 v) const {
        return x == v.x && y == v.y && z == v.z && w == v.w;
    }

    bool operator!=(Vector4 v) const {
        return !(*this == v);
    }

    float operator[](int index) const {
        return (&x)[index];
    }

    float& operator[](int index) {
        return (&x)[index];
    }
};

constexpr Vector4 Vector4_Zero = Vector4(0.f);

namespace std {
template<> struct hash<Vector4> {
    size_t operator()(Vector4 v) const {
        size_t hash = 0;
        hash_combine(hash, v.x);
        hash_combine(hash, v.y);
        hash_combine(hash, v.z);
        hash_combine(hash, v.w);
        return hash;
    }
};
}

inline Vector3::Vector3(const Vector4& v)
    : x(v.x), y(v.y), z(v.z) 
{}

//
// Vector2i
//
struct Vector2i {
    int x = 0, y = 0;

    bool operator==(Vector2i other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(Vector2i other) const {
        return !(*this == other);
    }
};

inline Vector2i operator+(Vector2i a, Vector2i b) {
    return Vector2i{ a.x + b.x, a.y + b.y };
}
inline Vector2i operator-(Vector2i a, Vector2i b) {
    return Vector2i{ a.x - b.x, a.y - b.y };
}
inline bool operator>(Vector2i a, Vector2i b) {
    return a.x > b.x && a.y > b.y;
}
inline bool operator>=(Vector2i a, Vector2i b) {
    return a.x >= b.x && a.y >= b.y;
}
inline bool operator<(Vector2i a, Vector2i b) {
    return a.x < b.x && a.y < b.y;
}
inline bool operator<=(Vector2i a, Vector2i b) {
    return a.x <= b.x && a.y <= b.y;
}

inline Vector2::Vector2(const Vector2i& v) {
    x = float(v.x);
    y = float(v.y);
}


//
// Vector3i
//
struct Vector3i {
    int x, y, z;

    Vector3i()
        : x(0), y(0), z(0) {}

    Vector3i(const Vector3& v)
        : x(int(v.x)), y(int(v.y)), z(int(v.z)) {}
};
