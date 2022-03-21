#include "std.h"
#include "lib/common.h"
#include "intersection_simd.h"

#include "lib/ray.h"

void Triangle_Intersection_8x::min(const Triangle_Intersection_8x& other)
{
    __m256 cmp_mask = _mm256_cmp_ps(other.t, t, _CMP_LT_OQ);
    t = _mm256_blendv_ps(t, other.t, cmp_mask);
    bx = _mm256_blendv_ps(bx, other.bx, cmp_mask);
    by = _mm256_blendv_ps(by, other.by, cmp_mask);
    bz = _mm256_blendv_ps(bz, other.bz, cmp_mask);
    triangle_index = _mm256_blendv_epi8(triangle_index, other.triangle_index, _mm256_castps_si256(cmp_mask));
}

void Triangle_Intersection_8x::reduce(float* distance, Vector3* barycentrics, uint32_t* primitive_index) const
{
    // reduce from 8 to 4 elements
    __m128 t_4, bx_4, by_4, bz_4;
    __m128i i_4;
    {
        __m128 t0_4 = _mm256_castps256_ps128(t);
        __m128 t1_4 = _mm256_castps256_ps128(_mm256_permute2f128_ps(t, t, 1));
        __m128 cmp_mask_4 = _mm_cmp_ps(t1_4, t0_4, _CMP_LT_OQ);
        t_4 = _mm_blendv_ps(t0_4, t1_4, cmp_mask_4);

        __m128 bx0_4 = _mm256_castps256_ps128(bx);
        __m128 bx1_4 = _mm256_castps256_ps128(_mm256_permute2f128_ps(bx, bx, 1));
        bx_4 = _mm_blendv_ps(bx0_4, bx1_4, cmp_mask_4);

        __m128 by0_4 = _mm256_castps256_ps128(by);
        __m128 by1_4 = _mm256_castps256_ps128(_mm256_permute2f128_ps(by, by, 1));
        by_4 = _mm_blendv_ps(by0_4, by1_4, cmp_mask_4);

        __m128 bz0_4 = _mm256_castps256_ps128(bz);
        __m128 bz1_4 = _mm256_castps256_ps128(_mm256_permute2f128_ps(bz, bz, 1));
        bz_4 = _mm_blendv_ps(bz0_4, bz1_4, cmp_mask_4);

        __m128i i0_4 = _mm256_castsi256_si128(triangle_index);
        __m128i i1_4 = _mm256_castsi256_si128(_mm256_permute2f128_si256(triangle_index, triangle_index, 1));
        i_4 = _mm_blendv_epi8(i0_4, i1_4, _mm_castps_si128(cmp_mask_4));
    }

    // reduce from 4 to 2 elements
    __m128 t_2, bx_2, by_2, bz_2;
    __m128i i_2;
    {
        __m128 t0_2 = t_4;
        __m128 t1_2 = _mm_shuffle_ps(t_4, t_4, 0xee);
        __m128 cmp_mask_2 = _mm_cmp_ps(t1_2, t0_2, _CMP_LT_OQ);
        t_2 = _mm_blendv_ps(t0_2, t1_2, cmp_mask_2);

        __m128 bx0_2 = bx_4;
        __m128 bx1_2 = _mm_shuffle_ps(bx_4, bx_4, 0xee);
        bx_2 = _mm_blendv_ps(bx0_2, bx1_2, cmp_mask_2);

        __m128 by0_2 = by_4;
        __m128 by1_2 = _mm_shuffle_ps(by_4, by_4, 0xee);
        by_2 = _mm_blendv_ps(by0_2, by1_2, cmp_mask_2);

        __m128 bz0_2 = bz_4;
        __m128 bz1_2 = _mm_shuffle_ps(bz_4, bz_4, 0xee);
        bz_2 = _mm_blendv_ps(bz0_2, bz1_2, cmp_mask_2);

        __m128i i0_2 = i_4;
        __m128i i1_2 = _mm_shuffle_epi32(i_4, 0xee);
        i_2 = _mm_blendv_epi8(i0_2, i1_2, _mm_castps_si128(cmp_mask_2));
    }

    // reduce from 2 to 1 element
    __m128 t_1, bx_1, by_1, bz_1;
    __m128i i_1;
    {
        __m128 t0 = t_2;
        __m128 t1 = _mm_shuffle_ps(t_2, t_2, 0xe5);
        __m128 cmp_mask_1 = _mm_cmp_ps(t1, t0, _CMP_LT_OQ);
        t_1 = _mm_blendv_ps(t0, t1, cmp_mask_1);

        __m128 bx0 = bx_2;
        __m128 bx1 = _mm_shuffle_ps(bx_2, bx_2, 0xe5);
        bx_1 = _mm_blendv_ps(bx0, bx1, cmp_mask_1);

        __m128 by0 = by_2;
        __m128 by1 = _mm_shuffle_ps(by_2, by_2, 0xe5);
        by_1 = _mm_blendv_ps(by0, by1, cmp_mask_1);

        __m128 bz0 = bz_2;
        __m128 bz1 = _mm_shuffle_ps(bz_2, bz_2, 0xe5);
        bz_1 = _mm_blendv_ps(bz0, bz1, cmp_mask_1);

        __m128i i0 = i_2;
        __m128i i1 = _mm_shuffle_epi32(i_2, 0xe5);
        i_1 = _mm_blendv_epi8(i0, i1, _mm_castps_si128(cmp_mask_1));
    }

    *distance = _mm_cvtss_f32(t_1);
    barycentrics->x = _mm_cvtss_f32(bx_1);
    barycentrics->y = _mm_cvtss_f32(by_1);
    barycentrics->z = _mm_cvtss_f32(bz_1);
    *primitive_index = (uint32_t)_mm_cvtsi128_si32(i_1);
}

Triangle_Intersection_8x intersect_triangle_watertight_8x(const Ray& ray, const __m256 px[3], const __m256 py[3], const __m256 pz[3])
{
    const int kz = ray.direction.abs().max_dimension();
    const int kx = (kz == 2 ? 0 : kz + 1);
    const int ky = (kz == 0 ? 2 : kz - 1);

    Vector3 direction = ray.direction.permutation(kx, ky, kz);
    float sx = -direction.x / direction.z;
    float sy = -direction.y / direction.z;
    float sz = 1.f / direction.z;

    __m256 p0t_z, x0, y0;
    __m256 p1t_z, x1, y1;
    __m256 p2t_z, x2, y2;
    {
        __m256 origin_x = _mm256_broadcast_ss(&ray.origin.x);
        __m256 origin_y = _mm256_broadcast_ss(&ray.origin.y);
        __m256 origin_z = _mm256_broadcast_ss(&ray.origin.z);

        float perm_masks[6] = { 0, 0,  -1, 0,  0, -1 };
        __m256 x_perm_mask0 = _mm256_set1_ps(perm_masks[2 * kx + 0]);
        __m256 x_perm_mask1 = _mm256_set1_ps(perm_masks[2 * kx + 1]);
        __m256 y_perm_mask0 = _mm256_set1_ps(perm_masks[2 * ky + 0]);
        __m256 y_perm_mask1 = _mm256_set1_ps(perm_masks[2 * ky + 1]);
        __m256 z_perm_mask0 = _mm256_set1_ps(perm_masks[2 * kz + 0]);
        __m256 z_perm_mask1 = _mm256_set1_ps(perm_masks[2 * kz + 1]);

        __m256 sx8 = _mm256_broadcast_ss(&sx);
        __m256 sy8 = _mm256_broadcast_ss(&sy);

        {
            __m256 p0t_xx = _mm256_sub_ps(px[0], origin_x);
            __m256 p0t_yy = _mm256_sub_ps(py[0], origin_y);
            __m256 p0t_zz = _mm256_sub_ps(pz[0], origin_z);
            __m256 p0t_x = _mm256_blendv_ps(_mm256_blendv_ps(p0t_xx, p0t_yy, x_perm_mask0), p0t_zz, x_perm_mask1);
            __m256 p0t_y = _mm256_blendv_ps(_mm256_blendv_ps(p0t_xx, p0t_yy, y_perm_mask0), p0t_zz, y_perm_mask1);
            p0t_z        = _mm256_blendv_ps(_mm256_blendv_ps(p0t_xx, p0t_yy, z_perm_mask0), p0t_zz, z_perm_mask1);
            x0 = _mm256_fmadd_ps(sx8, p0t_z, p0t_x);
            y0 = _mm256_fmadd_ps(sy8, p0t_z, p0t_y);
        }
        {
            __m256 p1t_xx = _mm256_sub_ps(px[1], origin_x);
            __m256 p1t_yy = _mm256_sub_ps(py[1], origin_y);
            __m256 p1t_zz = _mm256_sub_ps(pz[1], origin_z);
            __m256 p1t_x = _mm256_blendv_ps(_mm256_blendv_ps(p1t_xx, p1t_yy, x_perm_mask0), p1t_zz, x_perm_mask1);
            __m256 p1t_y = _mm256_blendv_ps(_mm256_blendv_ps(p1t_xx, p1t_yy, y_perm_mask0), p1t_zz, y_perm_mask1);
            p1t_z        = _mm256_blendv_ps(_mm256_blendv_ps(p1t_xx, p1t_yy, z_perm_mask0), p1t_zz, z_perm_mask1);
            x1 = _mm256_fmadd_ps(sx8, p1t_z, p1t_x);
            y1 = _mm256_fmadd_ps(sy8, p1t_z, p1t_y);
        }
        {
            __m256 p2t_xx = _mm256_sub_ps(px[2], origin_x);
            __m256 p2t_yy = _mm256_sub_ps(py[2], origin_y);
            __m256 p2t_zz = _mm256_sub_ps(pz[2], origin_z);
            __m256 p2t_x = _mm256_blendv_ps(_mm256_blendv_ps(p2t_xx, p2t_yy, x_perm_mask0), p2t_zz, x_perm_mask1);
            __m256 p2t_y = _mm256_blendv_ps(_mm256_blendv_ps(p2t_xx, p2t_yy, y_perm_mask0), p2t_zz, y_perm_mask1);
            p2t_z        = _mm256_blendv_ps(_mm256_blendv_ps(p2t_xx, p2t_yy, z_perm_mask0), p2t_zz, z_perm_mask1);
            x2 = _mm256_fmadd_ps(sx8, p2t_z, p2t_x);
            y2 = _mm256_fmadd_ps(sy8, p2t_z, p2t_y);
        }
    }

    __m256 e0 = _mm256_mul_ps(y1, x2);
    __m256 e0_err = _mm256_fmsub_ps(y1, x2, e0);
    e0 = _mm256_fmsub_ps(x1, y2, e0);
    e0 = _mm256_sub_ps(e0, e0_err);

    __m256 e1 = _mm256_mul_ps(y2, x0);
    __m256 e1_err = _mm256_fmsub_ps(y2, x0, e1);
    e1 = _mm256_fmsub_ps(x2, y0, e1);
    e1 = _mm256_sub_ps(e1, e1_err);

    __m256 e2 = _mm256_mul_ps(y0, x1);
    __m256 e2_err = _mm256_fmsub_ps(y0, x1, e2);
    e2 = _mm256_fmsub_ps(x0, y1, e2);
    e2 = _mm256_sub_ps(e2, e2_err);

    // If any of e0/e1/e2 is zero then re-do computation in double precision.
    int is_zero_edge;
    {
        __m256i mask = _mm256_set1_epi32(0x7fffffff); // all bits except sign bit, so we detect both +0 and -0
        is_zero_edge  = _mm256_testz_si256(_mm256_castps_si256(e0), mask);
        is_zero_edge |= _mm256_testz_si256(_mm256_castps_si256(e1), mask);
        is_zero_edge |= _mm256_testz_si256(_mm256_castps_si256(e2), mask);
    }
    if (is_zero_edge) {
        __m256d x0_lo = _mm256_cvtps_pd(_mm256_castps256_ps128(x0));
        __m256d x0_hi = _mm256_cvtps_pd(_mm256_castps256_ps128(_mm256_permute2f128_ps(x0, x0, 0x5)));
        __m256d x1_lo = _mm256_cvtps_pd(_mm256_castps256_ps128(x1));
        __m256d x1_hi = _mm256_cvtps_pd(_mm256_castps256_ps128(_mm256_permute2f128_ps(x1, x1, 0x5)));
        __m256d x2_lo = _mm256_cvtps_pd(_mm256_castps256_ps128(x2));
        __m256d x2_hi = _mm256_cvtps_pd(_mm256_castps256_ps128(_mm256_permute2f128_ps(x2, x2, 0x5)));
        __m256d y0_lo = _mm256_cvtps_pd(_mm256_castps256_ps128(y0));
        __m256d y0_hi = _mm256_cvtps_pd(_mm256_castps256_ps128(_mm256_permute2f128_ps(y0, y0, 0x5)));
        __m256d y1_lo = _mm256_cvtps_pd(_mm256_castps256_ps128(y1));
        __m256d y1_hi = _mm256_cvtps_pd(_mm256_castps256_ps128(_mm256_permute2f128_ps(y1, y1, 0x5)));
        __m256d y2_lo = _mm256_cvtps_pd(_mm256_castps256_ps128(y2));
        __m256d y2_hi = _mm256_cvtps_pd(_mm256_castps256_ps128(_mm256_permute2f128_ps(y2, y2, 0x5)));

        __m256d e0_lo = _mm256_mul_pd(y1_lo, x2_lo);
        e0_lo = _mm256_fmsub_pd(x1_lo, y2_lo, e0_lo);
        __m256d e0_hi = _mm256_mul_pd(y1_hi, x2_hi);
        e0_hi = _mm256_fmsub_pd(x1_hi, y2_hi, e0_hi);
        e0 = _mm256_set_m128(_mm256_cvtpd_ps(e0_hi), _mm256_cvtpd_ps(e0_lo));

        __m256d e1_lo = _mm256_mul_pd(y2_lo, x0_lo);
        e1_lo = _mm256_fmsub_pd(x2_lo, y0_lo, e1_lo);
        __m256d e1_hi = _mm256_mul_pd(y2_hi, x0_hi);
        e1_hi = _mm256_fmsub_pd(x2_hi, y0_hi, e1_hi);
        e1 = _mm256_set_m128(_mm256_cvtpd_ps(e1_hi), _mm256_cvtpd_ps(e1_lo));

        __m256d e2_lo = _mm256_mul_pd(y0_lo, x1_lo);
        e2_lo = _mm256_fmsub_pd(x0_lo, y1_lo, e2_lo);
        __m256d e2_hi = _mm256_mul_pd(y0_hi, x1_hi);
        e2_hi = _mm256_fmsub_pd(x0_hi, y1_hi, e2_hi);
        e2 = _mm256_set_m128(_mm256_cvtpd_ps(e2_hi), _mm256_cvtpd_ps(e2_lo));
    }

    __m256 zero = _mm256_setzero_ps();

    // Check if edge values have the same side. If it's not the case then there is no intersection.
    __m256 disabled_lanes_mask;
    {
        __m256 lt_zero = _mm256_cmp_ps(e0, zero, _CMP_LT_OQ);
        lt_zero = _mm256_or_ps(lt_zero, _mm256_cmp_ps(e1, zero, _CMP_LT_OQ));
        lt_zero = _mm256_or_ps(lt_zero, _mm256_cmp_ps(e2, zero, _CMP_LT_OQ));

        __m256 gt_zero = _mm256_cmp_ps(e0, zero, _CMP_GT_OQ);
        gt_zero = _mm256_or_ps(gt_zero, _mm256_cmp_ps(e1, zero, _CMP_GT_OQ));
        gt_zero = _mm256_or_ps(gt_zero, _mm256_cmp_ps(e2, zero, _CMP_GT_OQ));

        disabled_lanes_mask = _mm256_and_ps(lt_zero, gt_zero);
        if (_mm256_movemask_ps(disabled_lanes_mask) == 255)
            return Triangle_Intersection_8x::no_intersection();
    }

    // Compute: det = e0 + e1 + e2
    __m256 det;
    {
        det = _mm256_add_ps(e0, e1);
        det = _mm256_add_ps(det, e2);

        __m256 det_is_zero = _mm256_cmp_ps(det, zero, _CMP_EQ_OQ);
        disabled_lanes_mask = _mm256_or_ps(disabled_lanes_mask, det_is_zero);
        if (_mm256_movemask_ps(disabled_lanes_mask) == 255)
            return Triangle_Intersection_8x::no_intersection();
    }

    // Compute: t_scaled
    __m256 t_scaled;
    {
        __m256 sz8 = _mm256_broadcast_ss(&sz);
        __m256 z0 = _mm256_mul_ps(sz8, p0t_z);
        __m256 z1 = _mm256_mul_ps(sz8, p1t_z);
        __m256 z2 = _mm256_mul_ps(sz8, p2t_z);

        t_scaled = _mm256_mul_ps(e0, z0);
        t_scaled = _mm256_fmadd_ps(e1, z1, t_scaled);
        t_scaled = _mm256_fmadd_ps(e2, z2, t_scaled);
    }

    // Check if there is no intersection based on signs of det and t_scaled.
    {
        __m256i sign_bits_mask = _mm256_set1_epi32(0x80000000);

        __m256i xored_sign_bits = _mm256_castps_si256(_mm256_xor_ps(det, t_scaled));
        xored_sign_bits = _mm256_and_epi32(xored_sign_bits, sign_bits_mask);

        __m256i opposite_signs_mask = _mm256_cmpeq_epi32(xored_sign_bits, sign_bits_mask);
        disabled_lanes_mask = _mm256_or_ps(disabled_lanes_mask, _mm256_castsi256_ps(opposite_signs_mask));
        if (_mm256_movemask_ps(disabled_lanes_mask) == 255)
            return Triangle_Intersection_8x::no_intersection();
    }

    __m256 all_infinity = _mm256_set1_ps(Infinity);
    __m256 all_ones = _mm256_set1_ps(1.f);

    __m256 inv_det = _mm256_div_ps(all_ones, det);
    __m256 t = _mm256_mul_ps(inv_det, t_scaled);

    Triangle_Intersection_8x intersection;
    intersection.t = _mm256_blendv_ps(t, all_infinity, disabled_lanes_mask);
    intersection.bx = _mm256_mul_ps(e0, inv_det);
    intersection.by = _mm256_mul_ps(e1, inv_det);
    intersection.bz = _mm256_mul_ps(e2, inv_det);
    return intersection;
}
