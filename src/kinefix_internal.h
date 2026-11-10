#ifndef KINEFIX_INTERNAL_H
#define KINEFIX_INTERNAL_H

#include "kinefix/kinefix.h"

/* Slots retain tombstones, so an application id never revives an old handle. */
typedef kf_object_snapshot_t kf_object_slot_t;
typedef kf_shape_snapshot_t kf_shape_slot_t;
typedef kf_constraint_snapshot_t kf_constraint_slot_t;

typedef struct kf_index_leaf_t
{
    size_t shape_index;
    kf_aabb_t bounds;
} kf_index_leaf_t;

typedef struct kf_index_node_t
{
    kf_aabb_t bounds;
    size_t left;
    size_t right;
    size_t shape_index;
    uint32_t object_id; /* homogeneous subtree owner, zero for mixed owners */
} kf_index_node_t;

/* Unit quaternion with an explicit scalar part, Q2.30. */
typedef struct kf_quat_t
{
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t w;
} kf_quat_t;

/* Row-major rotation matrix, Q2.30. */
typedef struct kf_matrix_t
{
    int32_t m[3][3];
} kf_matrix_t;

/* Symmetric tensor with a 64-bit range: shape inertia per unit mass is Q8.24;
   inverse inertia used by the solver is Q32.32. */
typedef struct kf_tensor_t
{
    int64_t xx;
    int64_t yy;
    int64_t zz;
    int64_t xy;
    int64_t xz;
    int64_t yz;
} kf_tensor_t;

/* Mass properties in the body frame; derived from the descriptor and shapes. */
typedef struct kf_mass_t
{
    int64_t inverse_mass;
    kf_vec3_t center;
    kf_tensor_t inverse_inertia;
    kf_fixed_t extent;
    kf_fixed_t thickness;
} kf_mass_t;

/* Actual center and world-space rotation vector (Q2.30) of one solver substep. The
   absolute tick fraction makes a partial segment after an impact explicit. */
typedef struct kf_rotation_segment_t
{
    kf_vec3_t center;
    uint64_t time;
    kf_quat_t rotation;
    int64_t vector[3];
} kf_rotation_segment_t;

/* Pose path of a center c(t) = start + (end - start - curve) t + curve t^2, with
   the rotation interpolated around it. A constant acceleration a over the path
   duration T gives curve = a T^2 / 2. */
typedef struct kf_motion_t
{
    kf_vec3_t center_start;
    kf_vec3_t center_end;
    kf_vec3_t center_curve;
    kf_quat_t rotation_start;
    kf_quat_t rotation_end;
    kf_vec3_t local_center;
    kf_fixed_t extent;
    const kf_rotation_segment_t * translation_segments;
    uint32_t translation_count;
    const kf_rotation_segment_t * rotation_segments;
    uint32_t rotation_count;
    uint64_t rotation_fraction_start;
} kf_motion_t;

typedef struct kf_solver_body_t kf_solver_body_t;
typedef struct kf_solver_manifold_t kf_solver_manifold_t;
typedef struct kf_solver_joint_t kf_solver_joint_t;
typedef struct kf_solver_path_t kf_solver_path_t;

struct kf_world_t
{
    kf_allocator_t allocator;
    kf_world_config_t config;
    kf_capacity_t capacity;
    kf_object_slot_t * objects; /* ascending desc.id, including tombstones */
    size_t object_count;
    kf_shape_slot_t * shapes; /* ascending object_id, then desc.id */
    size_t shape_count;
    kf_constraint_slot_t * constraints; /* ascending desc.id, including tombstones */
    size_t constraint_count;
    kf_index_leaf_t * leaves;
    kf_index_node_t * nodes;
    size_t node_count;
    kf_bool_t index_dirty;
    uint64_t tick;
    size_t * shape_objects; /* shape-owner cache for dynamics */
    kf_aabb_t * scratch_bounds; /* canonical-order bounds during dynamics */
    size_t * candidates; /* sorted broad-phase candidates during dynamics */
    kf_solver_body_t * bodies;
    kf_solver_path_t * paths; /* per-body substep history, valid during a step */
    kf_solver_manifold_t * manifolds;
    size_t manifold_count;
    kf_solver_joint_t * joints;
    kf_contact_snapshot_t * cache; /* canonical contact impulses of the last step */
    size_t cache_count;
    kf_contact_snapshot_t * next_cache; /* built during a step, swapped on commit */
    /* Derived caches, recomputed from authoritative state when marked stale. */
    kf_aabb_t * shape_bounds; /* current bounds, in canonical shape order */
    kf_bool_t bounds_dirty;
    kf_mass_t * masses; /* body mass properties, in canonical object order */
    kf_bool_t masses_dirty;
    kf_bool_t reports_valid; /* manifolds still index the last step's bodies */
    kf_trigger_overlap_t * triggers; /* canonical overlaps after the last step */
    size_t trigger_count;
    kf_trigger_overlap_t * previous_triggers; /* overlaps before the last step */
    size_t previous_trigger_count;
    size_t trigger_overflow; /* overlaps found by a step that exceeded the capacity */
    kf_bool_t triggers_valid; /* both overlap sets come from the last step */
};

kf_object_slot_t * kf_internal_object( kf_world_t * world, kf_handle_t handle );
const kf_object_slot_t * kf_internal_object_const( const kf_world_t * world, kf_handle_t handle );
kf_object_slot_t * kf_internal_object_id( kf_world_t * world, uint32_t id );
kf_status_t kf_internal_validate_body( const kf_body_desc_t * desc );
kf_status_t kf_internal_validate_config( const kf_world_config_t * config );
kf_bool_t kf_internal_bounds_overlap( kf_aabb_t left, kf_aabb_t right );
void kf_internal_sort_leaves( kf_index_leaf_t * leaves, size_t count );
/* Builds the balanced tree over the sorted leaves. */
void kf_internal_build_tree( kf_world_t * world );
/* Marks the index, every derived cache and the contact reports stale after a
   structural change. */
void kf_internal_invalidate( kf_world_t * world );
kf_status_t kf_internal_refresh_bounds( kf_world_t * world );
kf_status_t kf_internal_update_object_bounds( kf_world_t * world, const kf_object_slot_t * object );
/* Canonical range [begin, end) of an object's shapes. */
void kf_internal_shape_range( const kf_world_t * world, uint32_t id, size_t * begin, size_t * end );
/* Strict canonical order of carried contact keys. */
kf_bool_t kf_internal_contact_less( const kf_contact_snapshot_t * left, const kf_contact_snapshot_t * right );
/* Canonical order of trigger overlaps by ids, then generations: -1, 0 or 1. */
int32_t kf_internal_trigger_compare( const kf_trigger_overlap_t * left, const kf_trigger_overlap_t * right );
kf_status_t kf_internal_vec_add( kf_vec3_t left, kf_vec3_t right, kf_vec3_t * result );
size_t kf_internal_solver_body_size( void );
size_t kf_internal_solver_manifold_size( void );
size_t kf_internal_solver_joint_size( void );
size_t kf_internal_solver_path_size( void );

/* Mass properties of a body from its descriptor and the world's canonical shapes. */
kf_status_t kf_internal_body_mass( const kf_world_t * world, const kf_object_slot_t * object, kf_mass_t * mass );
kf_vec3_t kf_internal_body_center( const kf_object_slot_t * object, const kf_mass_t * mass );
/* Rejects a target whose required center/angular speed exceeds the step limits. */
kf_status_t kf_internal_validate_kinematic_target( const kf_world_config_t * config, const kf_body_state_t * state, const kf_mass_t * mass );

#if defined(__SIZEOF_INT128__) && !defined(KF_PORTABLE_WIDE)
#   define KF_INTERNAL_NATIVE_WIDE 1
__extension__ typedef unsigned __int128 kf_wide_t;
#else
/* MSVC has no 128-bit integer type; its intrinsics give the same exact results. */
#   if defined(_MSC_VER) && !defined(KF_PORTABLE_WIDE)
#       if defined(_M_X64) && !defined(_M_ARM64EC)
#           include <intrin.h>
#           define KF_INTERNAL_MSVC_X64 1
#       elif defined(_M_ARM64)
#           include <intrin.h>
#           define KF_INTERNAL_MSVC_ARM64 1
#       endif
#   endif
typedef struct kf_wide_t
{
    uint64_t high;
    uint64_t low;
} kf_wide_t;
#endif

#define KF_INTERNAL_PI INT64_C(3373259426) /* Q2.30 */

KF_FORCE_INLINE uint64_t kf_internal_magnitude( int64_t value )
{
    return value < 0 ? (uint64_t)(-(value + 1)) + UINT64_C(1) : (uint64_t)value;
}

KF_FORCE_INLINE int64_t kf_internal_signed( uint64_t magnitude, kf_bool_t negative )
{
    if( magnitude > (uint64_t)INT64_MAX )
    {
        magnitude = (uint64_t)INT64_MAX;
    }
    return negative == KF_TRUE ? -(int64_t)magnitude : (int64_t)magnitude;
}

KF_FORCE_INLINE kf_wide_t kf_internal_wide_multiply( uint64_t left, uint64_t right )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    return (kf_wide_t)left * (kf_wide_t)right;
#elif defined(KF_INTERNAL_MSVC_X64)
    kf_wide_t result;
    result.low = _umul128( left, right, &result.high );
    return result;
#elif defined(KF_INTERNAL_MSVC_ARM64)
    kf_wide_t result;
    result.high = __umulh( left, right );
    result.low = left * right;
    return result;
#else
    uint64_t l0 = (uint32_t)left;
    uint64_t l1 = left >> 32;
    uint64_t r0 = (uint32_t)right;
    uint64_t r1 = right >> 32;
    uint64_t p00 = l0 * r0;
    uint64_t p01 = l0 * r1;
    uint64_t p10 = l1 * r0;
    uint64_t p11 = l1 * r1;
    uint64_t middle = (p00 >> 32) + (uint32_t)p01 + (uint32_t)p10;
    kf_wide_t result;
    result.high = p11 + (p01 >> 32) + (p10 >> 32) + (middle >> 32);
    result.low = (p00 & UINT64_C(0xFFFFFFFF)) | (middle << 32);
    return result;
#endif
}

KF_FORCE_INLINE void kf_internal_wide_parts( kf_wide_t value, uint64_t * high, uint64_t * low )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    *high = (uint64_t)(value >> 64);
    *low = (uint64_t)value;
#else
    *high = value.high;
    *low = value.low;
#endif
}

/* Exact unsigned magnitudes. Callers bound sums below 2^128 and only subtract
   a smaller magnitude from a larger one. */
KF_FORCE_INLINE kf_wide_t kf_internal_wide_make( uint64_t high, uint64_t low )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    return ( (kf_wide_t)high << 64 ) | low;
#else
    kf_wide_t result = {high, low};
    return result;
#endif
}

KF_FORCE_INLINE int32_t kf_internal_wide_compare( kf_wide_t left, kf_wide_t right )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    return left == right ? 0 : ( left < right ? -1 : 1 );
#else
    if( left.high != right.high )
    {
        return left.high < right.high ? -1 : 1;
    }
    return left.low == right.low ? 0 : ( left.low < right.low ? -1 : 1 );
#endif
}

KF_FORCE_INLINE kf_wide_t kf_internal_wide_add( kf_wide_t left, kf_wide_t right )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    return left + right;
#else
    kf_wide_t result;
    result.low = left.low + right.low;
    result.high = left.high + right.high + ( result.low < left.low ? 1u : 0u );
    return result;
#endif
}

KF_FORCE_INLINE kf_wide_t kf_internal_wide_subtract( kf_wide_t left, kf_wide_t right )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    return left - right;
#else
    kf_wide_t result;
    result.low = left.low - right.low;
    result.high = left.high - right.high - ( left.low < right.low ? 1u : 0u );
    return result;
#endif
}

/* Adds a signed value to a sign and magnitude accumulator; callers keep every
   magnitude below 2^128. */
KF_FORCE_INLINE void kf_internal_wide_accumulate( kf_wide_t * magnitude, kf_bool_t * negative, kf_wide_t value, kf_bool_t value_negative )
{
    if( *negative == value_negative )
    {
        *magnitude = kf_internal_wide_add( *magnitude, value );
        return;
    }
    if( kf_internal_wide_compare( *magnitude, value ) >= 0 )
    {
        *magnitude = kf_internal_wide_subtract( *magnitude, value );
        return;
    }
    *magnitude = kf_internal_wide_subtract( value, *magnitude );
    *negative = value_negative;
}

/* Shifts below 128 bits; values above the limit, at most INT64_MAX, saturate. */
KF_FORCE_INLINE uint64_t kf_internal_wide_shift( kf_wide_t value, uint32_t shift, uint64_t limit )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    kf_wide_t shifted = value >> shift;
    if( shifted > (kf_wide_t)limit )
    {
        return limit;
    }
    return (uint64_t)shifted;
#else
    uint64_t high;
    uint64_t low;
    if( shift >= 64u )
    {
        high = 0;
        low = shift == 64u ? value.high : value.high >> (shift - 64u);
    }
    else if( shift == 0 )
    {
        high = value.high;
        low = value.low;
    }
    else
    {
        high = value.high >> shift;
        low = (value.low >> shift) | (value.high << (64u - shift));
    }
    if( high != 0 || low > limit )
    {
        return limit;
    }
    return low;
#endif
}

/* Symmetric saturation keeps every result safe to negate. */
KF_FORCE_INLINE int64_t kf_internal_add_saturated( int64_t _left, int64_t _right )
{
    if( _right > 0 && _left > INT64_MAX - _right )
    {
        return INT64_MAX;
    }
    if( _right < 0 && _left < -INT64_MAX - _right )
    {
        return -INT64_MAX;
    }
    return _left + _right;
}

KF_FORCE_INLINE int64_t kf_internal_sub_saturated( int64_t _left, int64_t _right )
{
    if( _right > 0 && _left < -INT64_MAX + _right )
    {
        return -INT64_MAX;
    }
    if( _right < 0 && _left > INT64_MAX + _right )
    {
        return INT64_MAX;
    }
    return _left - _right;
}

/* Signed product shifted right, with a 128-bit intermediate. The result
   truncates toward zero and saturates to the int64 range. */
KF_FORCE_INLINE int64_t kf_internal_mul_shift( int64_t left, int64_t right, uint32_t shift )
{
    kf_bool_t negative = (kf_bool_t)((left < 0) != (right < 0));
    return kf_internal_signed( kf_internal_wide_shift( kf_internal_wide_multiply( kf_internal_magnitude( left ), kf_internal_magnitude( right ) ), shift, (uint64_t)INT64_MAX ), negative );
}

/* kf_internal_mul_shift saturated to +/-2^61 instead, so that sums of up to three
   terms cannot overflow; results below the limit are identical. */
KF_FORCE_INLINE int64_t kf_internal_mul_shift_term( int64_t left, int64_t right, uint32_t shift )
{
    kf_bool_t negative = (kf_bool_t)((left < 0) != (right < 0));
    return kf_internal_signed( kf_internal_wide_shift( kf_internal_wide_multiply( kf_internal_magnitude( left ), kf_internal_magnitude( right ) ), shift, UINT64_C(1) << 61 ), negative );
}

/* Signed left * middle * right >> shift from one 192-bit product, truncating
   toward zero and saturating to the int64 range; shift is below 128. */
KF_FORCE_INLINE int64_t kf_internal_mul3_shift( int64_t left, int64_t middle, int64_t right, uint32_t shift )
{
    kf_bool_t negative = (kf_bool_t)(((left < 0) != (middle < 0)) != (right < 0));
    uint64_t magnitude = kf_internal_magnitude( right );
    uint64_t high;
    uint64_t low;
    uint64_t upper_high;
    uint64_t upper_low;
    uint64_t lower_high;
    uint64_t lower_low;
    uint64_t words[3];
    uint64_t result;
    uint64_t overflow;
    kf_internal_wide_parts( kf_internal_wide_multiply( kf_internal_magnitude( left ), kf_internal_magnitude( middle ) ), &high, &low );
    kf_internal_wide_parts( kf_internal_wide_multiply( low, magnitude ), &lower_high, &lower_low );
    kf_internal_wide_parts( kf_internal_wide_multiply( high, magnitude ), &upper_high, &upper_low );
    words[0] = lower_low;
    words[1] = upper_low + lower_high;
    words[2] = upper_high + (words[1] < upper_low ? 1u : 0u);
    if( shift >= 64u )
    {
        uint32_t rest = shift - 64u;
        result = rest == 0 ? words[1] : (words[1] >> rest) | (words[2] << (64u - rest));
        overflow = rest == 0 ? words[2] : words[2] >> rest;
    }
    else if( shift == 0 )
    {
        result = words[0];
        overflow = words[1] | words[2];
    }
    else
    {
        result = (words[0] >> shift) | (words[1] << (64u - shift));
        overflow = (words[1] >> shift) | words[2];
    }
    if( overflow != 0 )
    {
        result = UINT64_MAX;
    }
    return kf_internal_signed( result, negative );
}

/* Quotient of the 128-bit value high:low by a divisor above high. */
uint64_t kf_internal_wide_divide( uint64_t high, uint64_t low, uint64_t divisor );
/* Full unsigned 128/128 division, truncating toward zero. A zero denominator
   or a quotient above UINT64_MAX returns UINT64_MAX. */
uint64_t kf_internal_wide_quotient( kf_wide_t numerator, kf_wide_t denominator );
/* Signed (left * right << shift) / denominator from one 128-bit numerator, with
   kf_internal_div_shift's rounding and saturation; shift is below 64. */
int64_t kf_internal_mul_div( int64_t left, int64_t right, int64_t denominator, uint32_t shift );
/* Signed (numerator << shift) / (first * second) with a 128-bit numerator and
   denominator, rounding and saturating likewise; shift is at most 64. */
int64_t kf_internal_div_product( int64_t numerator, int64_t first, int64_t second, uint32_t shift );
/* Signed (numerator << shift) / denominator with the same rounding and saturation;
   shift is below 64. */
int64_t kf_internal_div_shift( int64_t numerator, int64_t denominator, uint32_t shift );
uint64_t kf_internal_sqrt( uint64_t value );
/* Q2.30 sine and cosine of a Q2.30 radian angle. */
void kf_internal_sin_cos( int64_t radians, int32_t * sine, int32_t * cosine );

kf_quat_t kf_internal_quat( kf_rotation_t rotation );
kf_rotation_t kf_internal_rotation( kf_quat_t quat );
/* Canonical sign of an already normalized quaternion. */
kf_rotation_t kf_internal_rotation_unit( kf_quat_t quat );
kf_quat_t kf_internal_quat_normalize( kf_quat_t quat );
kf_quat_t kf_internal_quat_multiply( kf_quat_t left, kf_quat_t right );
kf_quat_t kf_internal_quat_conjugate( kf_quat_t quat );
/* Rotates by a world-axis rotation vector in Q2.30 radians. */
kf_quat_t kf_internal_quat_rotate( kf_quat_t quat, const int64_t vector[3] );
kf_quat_t kf_internal_quat_nlerp( kf_quat_t from, kf_quat_t to, uint64_t fraction );
/* Quaternion chord |to - from| after sign alignment, Q2.30. A point at distance r
   moves at most 2 * sqrt(2) * r * chord along the normalized interpolation. */
int64_t kf_internal_quat_chord( kf_quat_t from, kf_quat_t to );
/* Rotation vector of a quaternion, Q16 radians. */
void kf_internal_quat_rotation_vector( kf_quat_t quat, int64_t vector[3] );
kf_matrix_t kf_internal_matrix( kf_quat_t quat );
kf_vec3_t kf_internal_matrix_apply( const kf_matrix_t * matrix, kf_vec3_t vector );
kf_vec3_t kf_internal_matrix_apply_transpose( const kf_matrix_t * matrix, kf_vec3_t vector );
kf_tensor_t kf_internal_tensor_rotate( const kf_matrix_t * matrix, kf_tensor_t tensor );
kf_tensor_t kf_internal_tensor_inverse( kf_tensor_t tensor );
void kf_internal_tensor_apply( const kf_tensor_t * tensor, const int64_t vector[3], int64_t result[3] );

/* Variants of public shape queries for shapes already validated in a world. */
kf_status_t kf_internal_shape_bounds( const kf_shape_t * shape, kf_pose_t pose, kf_aabb_t * bounds );
kf_status_t kf_internal_shape_unbounded_bounds( const kf_shape_t * shape, kf_pose_t pose, kf_aabb_t * bounds );
kf_status_t kf_internal_shape_manifold( const kf_shape_t * left, kf_pose_t left_pose, const kf_shape_t * right, kf_pose_t right_pose, kf_fixed_t margin, kf_manifold_t * manifold );
/* Interpolated solver poses may extend slightly beyond the public world box. */
kf_status_t kf_internal_shape_unbounded_manifold( const kf_shape_t * left, kf_pose_t left_pose, const kf_shape_t * right, kf_pose_t right_pose, kf_fixed_t margin, kf_manifold_t * manifold );
/* KF_OK when valid shapes touch within kf_shape_contact's tolerance, otherwise KF_NO_HIT. */
kf_status_t kf_internal_shape_overlap( const kf_shape_t * left, kf_pose_t left_pose, const kf_shape_t * right, kf_pose_t right_pose );

/* Volume (Q16.16 cubic units), centroid and inertia per unit mass about the centroid. */
kf_status_t kf_internal_shape_mass( const kf_shape_t * shape, int64_t * volume, kf_vec3_t * centroid, kf_tensor_t * inertia, kf_fixed_t * thickness );
/* Farthest point of a shape from a body-local point. */
kf_status_t kf_internal_shape_extent( const kf_shape_t * shape, kf_vec3_t center, kf_fixed_t * extent );

kf_vec3_t kf_internal_motion_center( const kf_motion_t * motion, uint64_t fraction );
kf_quat_t kf_internal_motion_rotation( const kf_motion_t * motion, uint64_t fraction );
kf_pose_t kf_internal_motion_pose( const kf_motion_t * motion, uint64_t fraction );

/* Caller validates both actual trajectories; relative-space endpoints can extend
   outside world bounds. Internal impact fraction retains Q32 precision. */
kf_status_t kf_internal_shape_sweep_fraction( const kf_shape_t * moving, kf_pose_t pose, kf_vec3_t displacement, const kf_shape_t * obstacle, kf_pose_t obstacle_pose, kf_sweep_hit_t * hit, uint64_t * fraction );

/* Conservative advancement of two translating and rotating shapes until their
   separation reaches the target. Reports the separation at the path start and
   the first fraction where the shapes are within the target, or KF_NO_HIT. An
   exhausted iteration budget reports the last proven separated fraction. */
kf_status_t kf_internal_shape_toi( const kf_shape_t * moving, const kf_motion_t * motion, const kf_shape_t * obstacle, const kf_motion_t * obstacle_motion, kf_fixed_t target, uint64_t * fraction, kf_fixed_t * initial_separation );

#endif
