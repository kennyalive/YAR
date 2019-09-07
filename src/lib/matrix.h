#pragma once

#include "bounding_box.h"
#include "ray.h"
#include "vector.h"

struct Matrix3x4 {
    float a[3][4];
    static const Matrix3x4 identity;

    void set_column(int column_index, Vector3 c);
    void set_row(int row_index, Vector4 r);
    Vector3 get_column(int column) const;
    Vector4 get_row(int row) const;
    bool is_identity() const;
};

struct Matrix4x4 {
    float a[4][4];
    static const Matrix4x4 identity;
};

Matrix3x4 operator*(const Matrix3x4& m1, const Matrix3x4& m2);
Matrix4x4 operator*(const Matrix4x4& m1, const Matrix3x4& m2);

// rotate_[axis] functions premultiply a given matrix by corresponding rotation matrix.
Matrix3x4 rotate_x(const Matrix3x4& m, float angle);
Matrix3x4 rotate_y(const Matrix3x4& m, float angle);
Matrix3x4 rotate_z(const Matrix3x4& m, float angle);

// premultiplies the given matrix by translation transform
Matrix3x4 translate(const Matrix3x4& m, const Vector3& translation);

Matrix3x4 uniform_scale(const Matrix3x4& m, float scale);

// Computes world space->eye space transform that positions the camera at point 'from'
// and orients its direction towards the point 'to'. 'up' unit vector specifies reference up direction.
Matrix3x4 look_at_transform(Vector3 from, Vector3 to, Vector3 up);

// Computes traditional perspective matrix that transforms position vector (x,y,z,1) to
// obtain clip coordinates (xc, yc, zc, wc) that can be transformed to normalized deviced
// coordinates (NDC) by perspective division (xd, yd, zd) = (xc/wc, yc/wc, zc/wc).
// Eye-space z-axis points towards the viewer (OpenGL style), right-handed coordinate system.
// z coordinate is mapped to 0 and 1 for near and far planes correspondingly. y axis in NDC
// space points top-down with regard to eye space vertical direction (to match Vulkan viewport).
Matrix4x4 perspective_transform_opengl_z01(float fovy_radians, float aspect_ratio, float near, float far);

// Assumes that input matrix constains only rotation, translation and uniform scale.
Matrix3x4 get_inverted_transform(const Matrix3x4& m);

// Computes transform which when applied to mirrored geometry gives the same 
// result as at first transforming original object with original transform and then mirroring it.
// This helps when geometry handedness need to be changed. To do this the geometry is fliped around
// an axis and transform is computed by this function.
Matrix3x4 get_mirrored_transform(const Matrix3x4& m, int flip_axis);

bool is_transform_changes_handedness(const Matrix3x4& m);

Vector3 transform_point(const Matrix3x4& m, Vector3 p);
Vector3 transform_vector(const Matrix3x4& m, Vector3 v);
Ray transform_ray(const Matrix3x4& m, const Ray& ray);
Bounding_Box transform_bounding_box(const Matrix3x4& m, const Bounding_Box& bounds);
