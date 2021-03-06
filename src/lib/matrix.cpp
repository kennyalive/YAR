#include "std.h"
#include "common.h"
#include "matrix.h"

const Matrix3x4 Matrix3x4::identity = [] {
    Matrix3x4 m{};
    m.a[0][0] = m.a[1][1] = m.a[2][2] = 1.f;
    return m;
}();

const Matrix3x4 Matrix3x4::zero = [] {
    return Matrix3x4{};
}();

const Matrix4x4 Matrix4x4::identity = [] {
    Matrix4x4 m{};
    m.a[0][0] = m.a[1][1] = m.a[2][2] = m.a[3][3] = 1.f;
    return m;
}();

void Matrix3x4::set_column(int column_index, Vector3 c) {
    ASSERT(column_index >= 0 && column_index < 4);
    a[0][column_index] = c.x;
    a[1][column_index] = c.y;
    a[2][column_index] = c.z;
}

void Matrix3x4::set_row(int row_index, Vector4 r) {
    ASSERT(row_index >= 0 && row_index < 3);
    a[row_index][0] = r.x;
    a[row_index][1] = r.y;
    a[row_index][2] = r.z;
    a[row_index][3] = r.w;
}

Vector3 Matrix3x4::get_column(int c) const {
    ASSERT(c >= 0 && c < 4);
    return Vector3(a[0][c], a[1][c], a[2][c]);
}

Vector4 Matrix3x4::get_row(int row) const {
    ASSERT(row >= 0 && row < 3);
    return Vector4(a[row][0], a[row][1], a[row][2], a[row][3]);
}

bool Matrix3x4::is_identity() const {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            if (a[i][j] != (i == j ? 1.f : 0.f))
                return false;
    return true;
}

bool Matrix3x4::is_zero() const {
    const float* p = &a[0][0];
    for (int i = 0; i < 12; i++, p++)
        if (*p != 0.f)
            return false;
    return true;
}

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

Matrix4x4 operator*(const Matrix4x4& m1, const Matrix3x4& m2) {
    Matrix4x4 m;
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

    m.a[3][0] = m1.a[3][0]*m2.a[0][0] + m1.a[3][1]*m2.a[1][0] + m1.a[3][2]*m2.a[2][0];
    m.a[3][1] = m1.a[3][0]*m2.a[0][1] + m1.a[3][1]*m2.a[1][1] + m1.a[3][2]*m2.a[2][1];
    m.a[3][2] = m1.a[3][0]*m2.a[0][2] + m1.a[3][1]*m2.a[1][2] + m1.a[3][2]*m2.a[2][2];
    m.a[3][3] = m1.a[3][0]*m2.a[0][3] + m1.a[3][1]*m2.a[1][3] + m1.a[3][2]*m2.a[2][3] + m1.a[3][3];
    return m;
}

Matrix3x4 rotate_x(const Matrix3x4& m, float angle) {
    float cs = std::cos(angle);
    float sn = std::sin(angle);

    Matrix3x4 m2;
    m2.a[0][0] = m.a[0][0];
    m2.a[0][1] = m.a[0][1];
    m2.a[0][2] = m.a[0][2];
    m2.a[0][3] = m.a[0][3];

    m2.a[1][0] = cs*m.a[1][0] - sn*m.a[2][0];
    m2.a[1][1] = cs*m.a[1][1] - sn*m.a[2][1];
    m2.a[1][2] = cs*m.a[1][2] - sn*m.a[2][2];
    m2.a[1][3] = cs*m.a[1][3] - sn*m.a[2][3];

    m2.a[2][0] = sn*m.a[1][0] + cs*m.a[2][0];
    m2.a[2][1] = sn*m.a[1][1] + cs*m.a[2][1];
    m2.a[2][2] = sn*m.a[1][2] + cs*m.a[2][2];
    m2.a[2][3] = sn*m.a[1][3] + cs*m.a[2][3];
    return m2;
}

Matrix3x4 rotate_y(const Matrix3x4& m, float angle) {
    float cs = std::cos(angle);
    float sn = std::sin(angle);

    Matrix3x4 m2;
    m2.a[0][0] = cs*m.a[0][0] + sn*m.a[2][0];
    m2.a[0][1] = cs*m.a[0][1] + sn*m.a[2][1];
    m2.a[0][2] = cs*m.a[0][2] + sn*m.a[2][2];
    m2.a[0][3] = cs*m.a[0][3] + sn*m.a[2][3];

    m2.a[1][0] = m.a[1][0];
    m2.a[1][1] = m.a[1][1];
    m2.a[1][2] = m.a[1][2];
    m2.a[1][3] = m.a[1][3];

    m2.a[2][0] = -sn*m.a[0][0] + cs*m.a[2][0];
    m2.a[2][1] = -sn*m.a[0][1] + cs*m.a[2][1];
    m2.a[2][2] = -sn*m.a[0][2] + cs*m.a[2][2];
    m2.a[2][3] = -sn*m.a[0][3] + cs*m.a[2][3];
    return m2;
}

Matrix3x4 rotate_z(const Matrix3x4& m, float angle) {
    float cs = std::cos(angle);
    float sn = std::sin(angle);

    Matrix3x4 m2;
    m2.a[0][0] = cs*m.a[0][0] - sn*m.a[1][0];
    m2.a[0][1] = cs*m.a[0][1] - sn*m.a[1][1];
    m2.a[0][2] = cs*m.a[0][2] - sn*m.a[1][2];
    m2.a[0][3] = cs*m.a[0][3] - sn*m.a[1][3];

    m2.a[1][0] = sn*m.a[0][0] + cs*m.a[1][0];
    m2.a[1][1] = sn*m.a[0][1] + cs*m.a[1][1];
    m2.a[1][2] = sn*m.a[0][2] + cs*m.a[1][2];
    m2.a[1][3] = sn*m.a[0][3] + cs*m.a[1][3];
    
    m2.a[2][0] = m.a[2][0];
    m2.a[2][1] = m.a[2][1];
    m2.a[2][2] = m.a[2][2];
    m2.a[2][3] = m.a[2][3];
    return m2;
}

Matrix3x4 translate(const Matrix3x4& m, const Vector3& translation) {
    Matrix3x4 m2;
    m2.a[0][0] = m.a[0][0];
    m2.a[0][1] = m.a[0][1];
    m2.a[0][2] = m.a[0][2];
    m2.a[0][3] = m.a[0][3] + translation[0];

    m2.a[1][0] = m.a[1][0];
    m2.a[1][1] = m.a[1][1];
    m2.a[1][2] = m.a[1][2];
    m2.a[1][3] = m.a[1][3] + translation[1];

    m2.a[2][0] = m.a[2][0];
    m2.a[2][1] = m.a[2][1];
    m2.a[2][2] = m.a[2][2];
    m2.a[2][3] = m.a[2][3] + translation[2];
    return m2;
}

Matrix3x4 uniform_scale_transform(const Matrix3x4& m, float scale) {
    Matrix3x4 m2;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m2.a[i][j] = m.a[i][j] * scale;

    m2.a[0][3] = m.a[0][3]; 
    m2.a[1][3] = m.a[1][3];
    m2.a[2][3] = m.a[2][3];
    return m2;
}

Matrix3x4 scale_transform(const Matrix3x4& m, const Vector3& scale) {
    Matrix3x4 m2;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m2.a[i][j] = m.a[i][j] * scale[i];

    m2.a[0][3] = m.a[0][3]; 
    m2.a[1][3] = m.a[1][3];
    m2.a[2][3] = m.a[2][3];
    return m2;
}

Matrix3x4 look_at_transform(Vector3 from, Vector3 to, Vector3 up) {
    ASSERT(up.is_normalized());

    Vector3 f = to - from;
    float d = f.length();

    // degenerated cases, just return matrix with identity orientation
    if (d < 1e-5f || std::abs(dot(f, up) - 1.f) < 1e-3f) {
        Matrix3x4 m = Matrix3x4::identity;
        m.set_column(3, from);
        return m;
    }

    f /= d;
    Vector3 r = cross(f, up).normalized();
    Vector3 u = cross(r, f);

    Matrix3x4 m;
    m.set_row(0, Vector4(r, -dot(from, r)));
    m.set_row(1, Vector4(f, -dot(from, f)));
    m.set_row(2, Vector4(u, -dot(from, u)));
    return m;
}

Matrix4x4 perspective_transform_opengl_z01(float fovy_radians, float aspect_ratio, float near, float far) {
    float h = std::tan(fovy_radians/2.f) * near;
    float w = aspect_ratio * h;

    Matrix4x4 proj{};
    proj.a[0][0] = near / w;
    proj.a[1][1] = -near / h;
    proj.a[2][2] = -far / (far - near);
    proj.a[2][3] = -far*near / (far - near);
    proj.a[3][2] = -1.f;
    return proj;
}

Matrix3x4 get_inverse_transform(const Matrix3x4& m) {
    Vector3 scale = get_scale_from_transform(m);
    Vector3 inv_scale = Vector3(1) / scale;

    Vector3 x_axis = inv_scale.x * m.get_column(0);
    Vector3 y_axis = inv_scale.y * m.get_column(1);
    Vector3 z_axis = inv_scale.z * m.get_column(2);
    Vector3 origin = m.get_column(3);

    Matrix3x4 m_inv;
    m_inv.set_row(0, inv_scale.x * Vector4(x_axis, -dot(x_axis, origin)));
    m_inv.set_row(1, inv_scale.y * Vector4(y_axis, -dot(y_axis, origin)));
    m_inv.set_row(2, inv_scale.z * Vector4(z_axis, -dot(z_axis, origin)));
    return m_inv;
}

Matrix3x4 get_mirrored_transform(const Matrix3x4& m, int flip_axis) {
    // Mirrored transform = F * M * F
    // F - transform that flips along flip_axis axis.
    // For example F_y_axix =
    //   1  0  0
    //   0 -1  0
    //   0  0  1
    //
    Matrix3x4 m2 = m;
    m2.a[0][flip_axis] = -m2.a[0][flip_axis];
    m2.a[1][flip_axis] = -m2.a[1][flip_axis];
    m2.a[2][flip_axis] = -m2.a[2][flip_axis];

    m2.a[flip_axis][0] = -m2.a[flip_axis][0];
    m2.a[flip_axis][1] = -m2.a[flip_axis][1];
    m2.a[flip_axis][2] = -m2.a[flip_axis][2];
    m2.a[flip_axis][3] = -m2.a[flip_axis][3];
    return m2;
}

Vector3 get_scale_from_transform(const Matrix3x4& m) {
    Vector3 scale;
    for (int i = 0; i < 3; i++) {
        float axis_length = m.get_column(i).length();
        ASSERT(axis_length != 0.f);
        // If scale has small deviation from 1.0 then assume it's due to
        // rounding error and in that case force scale to be exactly 1.0.
        scale[i] = (std::abs(axis_length - 1.f) < 1e-6f) ? 1.f : axis_length;
    }
    return scale;
}

bool is_transform_changes_handedness(const Matrix3x4& m) {
    return  m.a[0][0] * (m.a[1][1]*m.a[2][2] - m.a[1][2]*m.a[2][1]) +
            m.a[0][1] * (m.a[1][2]*m.a[2][0] - m.a[1][0]*m.a[2][2]) +
            m.a[0][2] * (m.a[1][0]*m.a[2][1] - m.a[1][1]*m.a[2][0]) < 0.f;
}

Vector3 transform_point(const Matrix3x4& m, Vector3 p) {
    Vector3 p2;
    p2.x = m.a[0][0]*p.x + m.a[0][1]*p.y + m.a[0][2]*p.z + m.a[0][3];
    p2.y = m.a[1][0]*p.x + m.a[1][1]*p.y + m.a[1][2]*p.z + m.a[1][3];
    p2.z = m.a[2][0]*p.x + m.a[2][1]*p.y + m.a[2][2]*p.z + m.a[2][3];
    return p2;
}

Vector3 transform_vector(const Matrix3x4& m, Vector3 v) {
    Vector3 v2;
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

Bounding_Box transform_bounding_box(const Matrix3x4& m, const Bounding_Box& bounds) {
    Bounding_Box bounds2;
    bounds2.add_point(transform_point(m, Vector3(bounds.min_p.x, bounds.min_p.y, bounds.min_p.z)));
    bounds2.add_point(transform_point(m, Vector3(bounds.min_p.x, bounds.min_p.y, bounds.max_p.z)));
    bounds2.add_point(transform_point(m, Vector3(bounds.min_p.x, bounds.max_p.y, bounds.min_p.z)));
    bounds2.add_point(transform_point(m, Vector3(bounds.min_p.x, bounds.max_p.y, bounds.max_p.z)));
    bounds2.add_point(transform_point(m, Vector3(bounds.max_p.x, bounds.min_p.y, bounds.min_p.z)));
    bounds2.add_point(transform_point(m, Vector3(bounds.max_p.x, bounds.min_p.y, bounds.max_p.z)));
    bounds2.add_point(transform_point(m, Vector3(bounds.max_p.x, bounds.max_p.y, bounds.min_p.z)));
    bounds2.add_point(transform_point(m, Vector3(bounds.max_p.x, bounds.max_p.y, bounds.max_p.z)));
    return bounds2;
}
