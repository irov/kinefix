#ifndef KINEFIX_KINEFIX_H
#define KINEFIX_KINEFIX_H

#include <stddef.h>
#include <stdint.h>

#if !defined(KF_FORCE_INLINE)
#   if defined(_MSC_VER)
#       define KF_FORCE_INLINE static __forceinline
#   elif defined(__GNUC__) || defined(__clang__)
#       define KF_FORCE_INLINE static inline __attribute__((__always_inline__))
#   else
#       define KF_FORCE_INLINE static inline
#   endif
#endif

#define KF_VERSION_MAJOR 1
#define KF_VERSION_MINOR 4
#define KF_VERSION_PATCH 0

#define KF_FIXED_FRACTION_BITS 16
#define KF_FIXED_SCALE INT32_C(65536)
#define KF_FIXED_PI INT32_C(205887)
#define KF_FIXED_HALF_PI INT32_C(102943)
#define KF_FIXED_TWO_PI INT32_C(411774)

#define KF_ANGLE16_TURN UINT32_C(65536)
#define KF_ANGLE16_QUARTER UINT16_C(16384)
#define KF_ANGLE16_HALF UINT16_C(32768)
#define KF_ANGLE16_THREE_QUARTERS UINT16_C(49152)

typedef int32_t kf_fixed_t;
typedef uint16_t kf_angle16_t;
typedef int16_t kf_sangle16_t;
typedef uint8_t kf_bool_t;

#define KF_FALSE ((kf_bool_t)0)
#define KF_TRUE ((kf_bool_t)1)

typedef struct kf_vec3_t
{
    kf_fixed_t x;
    kf_fixed_t y;
    kf_fixed_t z;
} kf_vec3_t;

typedef struct kf_pcg32_t
{
    uint64_t state;
    uint64_t increment;
} kf_pcg32_t;

typedef struct kf_aabb_t
{
    kf_vec3_t minimum;
    kf_vec3_t maximum;
} kf_aabb_t;

typedef struct kf_sphere_t
{
    kf_vec3_t position;
    kf_fixed_t radius;
} kf_sphere_t;

/* Y-up character capsule. Position is the bottom of the capsule. */
typedef struct kf_capsule_t
{
    kf_vec3_t position;
    kf_fixed_t radius;
    kf_fixed_t height;
} kf_capsule_t;

typedef struct kf_ray_t
{
    kf_vec3_t origin;
    kf_vec3_t direction;
    kf_fixed_t maximum_distance;
} kf_ray_t;

kf_fixed_t kf_fixed_from_int( int64_t value );
kf_fixed_t kf_fixed_from_ratio( int64_t numerator, int64_t denominator );
kf_fixed_t kf_fixed_from_float( float value );
kf_fixed_t kf_fixed_from_double( double value );
int32_t kf_fixed_to_int( kf_fixed_t value );
float kf_fixed_to_float( kf_fixed_t value );
double kf_fixed_to_double( kf_fixed_t value );
kf_fixed_t kf_fixed_neg( kf_fixed_t value );
kf_fixed_t kf_fixed_add( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_sub( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_mul( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_div( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_add_mul( kf_fixed_t base, kf_fixed_t value, kf_fixed_t scalar );
kf_fixed_t kf_fixed_sub_mul( kf_fixed_t base, kf_fixed_t value, kf_fixed_t scalar );
kf_fixed_t kf_fixed_add_mul2( kf_fixed_t base, kf_fixed_t first, kf_fixed_t first_scalar, kf_fixed_t second, kf_fixed_t second_scalar );
kf_fixed_t kf_fixed_mul_div( kf_fixed_t value, kf_fixed_t multiplier, kf_fixed_t divisor );
kf_fixed_t kf_fixed_lerp( kf_fixed_t from, kf_fixed_t to, kf_fixed_t factor );

KF_FORCE_INLINE kf_fixed_t kf_fixed_abs( kf_fixed_t value )
{
    if( value == INT32_MIN )
    {
        return kf_fixed_neg( value );
    }

    return value < 0 ? -value : value;
}

KF_FORCE_INLINE kf_fixed_t kf_fixed_min( kf_fixed_t left, kf_fixed_t right )
{
    return left < right ? left : right;
}

KF_FORCE_INLINE kf_fixed_t kf_fixed_max( kf_fixed_t left, kf_fixed_t right )
{
    return left > right ? left : right;
}

KF_FORCE_INLINE kf_fixed_t kf_fixed_clamp( kf_fixed_t value, kf_fixed_t minimum, kf_fixed_t maximum )
{
    return kf_fixed_max( minimum, kf_fixed_min( maximum, value ) );
}

kf_fixed_t kf_fixed_sqrt( kf_fixed_t value );
kf_fixed_t kf_fixed_wrap_angle( kf_fixed_t angle );
void kf_fixed_sin_cos( kf_fixed_t angle, kf_fixed_t * sine, kf_fixed_t * cosine );

kf_angle16_t kf_angle16_from_fixed_radians( kf_fixed_t radians );
kf_sangle16_t kf_sangle16_from_fixed_radians( kf_fixed_t radians );
kf_fixed_t kf_angle16_to_fixed_radians( kf_angle16_t angle );
kf_fixed_t kf_sangle16_to_fixed_radians( kf_sangle16_t angle );
kf_angle16_t kf_angle16_add( kf_angle16_t angle, kf_sangle16_t delta );
kf_sangle16_t kf_sangle16_add_clamped( kf_sangle16_t angle, kf_sangle16_t delta, kf_sangle16_t minimum, kf_sangle16_t maximum );
void kf_angle16_sin_cos( kf_angle16_t angle, kf_fixed_t * sine, kf_fixed_t * cosine );
void kf_sangle16_sin_cos( kf_sangle16_t angle, kf_fixed_t * sine, kf_fixed_t * cosine );

kf_vec3_t kf_vec3_add( kf_vec3_t left, kf_vec3_t right );
kf_vec3_t kf_vec3_sub( kf_vec3_t left, kf_vec3_t right );
kf_vec3_t kf_vec3_neg( kf_vec3_t value );
kf_vec3_t kf_vec3_mul( kf_vec3_t value, kf_fixed_t scalar );
kf_vec3_t kf_vec3_div( kf_vec3_t value, kf_fixed_t scalar );
kf_vec3_t kf_vec3_add_mul( kf_vec3_t base, kf_vec3_t value, kf_fixed_t scalar );
kf_vec3_t kf_vec3_sub_mul( kf_vec3_t base, kf_vec3_t value, kf_fixed_t scalar );
kf_vec3_t kf_vec3_add_mul2( kf_vec3_t base, kf_vec3_t first, kf_fixed_t first_scalar, kf_vec3_t second, kf_fixed_t second_scalar );
kf_vec3_t kf_vec3_mul_div( kf_vec3_t value, kf_fixed_t multiplier, kf_fixed_t divisor );
kf_vec3_t kf_vec3_lerp( kf_vec3_t from, kf_vec3_t to, kf_fixed_t factor );
kf_bool_t kf_vec3_equal( kf_vec3_t left, kf_vec3_t right );
kf_fixed_t kf_vec3_dot( kf_vec3_t left, kf_vec3_t right );
kf_vec3_t kf_vec3_cross( kf_vec3_t left, kf_vec3_t right );
kf_fixed_t kf_vec3_length_squared( kf_vec3_t value );
kf_fixed_t kf_vec3_length( kf_vec3_t value );
kf_vec3_t kf_vec3_normalize( kf_vec3_t value );
kf_fixed_t kf_segment_point_distance_squared( kf_vec3_t start, kf_vec3_t end, kf_vec3_t point );

uint64_t kf_splitmix64( uint64_t value );
void kf_pcg32_seed( kf_pcg32_t * random, uint64_t seed, uint64_t stream );
uint32_t kf_pcg32_next( kf_pcg32_t * random );
uint32_t kf_pcg32_bounded( kf_pcg32_t * random, uint32_t bound );
void kf_pcg32_restore( kf_pcg32_t * random, uint64_t state, uint64_t stream );

#define KF_COORDINATE_LIMIT INT32_C(268435456) /* +/-4096 units */
#define KF_SIZE_LIMIT INT32_C(33554432)        /* 512 units */
#define KF_SPEED_LIMIT INT32_C(67108864)       /* 1024 units/second */
#define KF_PRISM_MAX_VERTICES 16u
#define KF_GEOMETRY_TOLERANCE INT32_C(8)
#define KF_MANIFOLD_MAX_POINTS 4u
#define KF_STATE_VERSION 2u

#define KF_ROTATION_FRACTION_BITS 30
#define KF_ROTATION_SCALE INT32_C(1073741824)

typedef enum kf_status_t
{
    KF_OK = 0,
    KF_NO_HIT,
    KF_INVALID_ARGUMENT,
    KF_OUT_OF_RANGE,
    KF_OVERFLOW,
    KF_CAPACITY_EXCEEDED,
    KF_OUT_OF_MEMORY,
    KF_STALE_HANDLE,
    KF_DUPLICATE_ID,
    KF_UNSUPPORTED,
    KF_ITERATION_LIMIT
} kf_status_t;

typedef struct kf_handle_t
{
    uint32_t id;
    uint32_t generation;
} kf_handle_t;

/* Vector part of a unit quaternion in Q2.30. The scalar part is the implicit
   nonnegative sqrt(1 - x*x - y*y - z*z), so a zero rotation is the identity. */
typedef struct kf_rotation_t
{
    int32_t x;
    int32_t y;
    int32_t z;
} kf_rotation_t;

typedef struct kf_pose_t
{
    kf_vec3_t position;
    kf_rotation_t rotation;
} kf_pose_t;

kf_bool_t kf_rotation_valid( kf_rotation_t rotation );
kf_rotation_t kf_rotation_from_yaw( kf_angle16_t yaw );
kf_rotation_t kf_rotation_from_axis_angle( kf_vec3_t axis, kf_fixed_t radians );
kf_rotation_t kf_rotation_multiply( kf_rotation_t left, kf_rotation_t right );
kf_rotation_t kf_rotation_inverse( kf_rotation_t rotation );
kf_vec3_t kf_rotation_apply( kf_rotation_t rotation, kf_vec3_t vector );
kf_vec3_t kf_rotation_apply_inverse( kf_rotation_t rotation, kf_vec3_t vector );

typedef enum kf_shape_type_t
{
    KF_SHAPE_AABB = 0,
    KF_SHAPE_SPHERE,
    KF_SHAPE_CAPSULE,
    KF_SHAPE_BOX,
    KF_SHAPE_PRISM,
    KF_SHAPE_TRIANGLE
} kf_shape_type_t;

typedef struct kf_box_t
{
    kf_vec3_t center;
    kf_vec3_t half_extents;
    kf_angle16_t yaw;
} kf_box_t;

/* Strict convex XZ polygon in either consistent winding, extruded in Y. */
typedef struct kf_prism_t
{
    kf_vec3_t vertices[KF_PRISM_MAX_VERTICES]; /* y must be zero */
    uint32_t count;
    kf_fixed_t minimum_y;
    kf_fixed_t maximum_y;
} kf_prism_t;

typedef struct kf_triangle_t
{
    kf_vec3_t vertices[3]; /* double-sided, static only in a world */
} kf_triangle_t;

typedef struct kf_shape_t
{
    kf_shape_type_t type;
    union
    {
        kf_aabb_t aabb;
        kf_sphere_t sphere;
        kf_capsule_t capsule;
        kf_box_t box;
        kf_prism_t prism;
        kf_triangle_t triangle;
    } data;
} kf_shape_t;

typedef struct kf_contact_t
{
    kf_vec3_t normal; /* from right/obstacle toward left/moving shape */
    kf_fixed_t depth; /* touching is zero */
    kf_vec3_t point_left;
    kf_vec3_t point_right;
    kf_handle_t object;
    uint32_t shape_id;
} kf_contact_t;

typedef struct kf_manifold_point_t
{
    kf_vec3_t point_left;
    kf_vec3_t point_right;
    kf_fixed_t separation; /* negative is penetration */
} kf_manifold_point_t;

typedef struct kf_manifold_t
{
    kf_vec3_t normal; /* from right toward left */
    uint32_t count;
    kf_manifold_point_t points[KF_MANIFOLD_MAX_POINTS];
} kf_manifold_t;

typedef struct kf_sweep_hit_t
{
    kf_handle_t object;
    uint32_t shape_id;
    kf_fixed_t fraction;
    uint64_t fraction_q32; /* exact ordering key in [0, 2^32] */
    kf_fixed_t distance;
    kf_vec3_t position;      /* moving pose.position at impact */
    kf_vec3_t contact_point; /* on obstacle surface */
    kf_vec3_t normal;
    kf_bool_t started_inside;
    kf_bool_t trigger;
} kf_sweep_hit_t;

kf_status_t kf_shape_validate( const kf_shape_t * shape );
kf_status_t kf_shape_bounds( const kf_shape_t * shape, kf_pose_t pose, kf_aabb_t * bounds );
kf_status_t kf_shape_contact( const kf_shape_t * left, kf_pose_t left_pose, const kf_shape_t * right, kf_pose_t right_pose, kf_contact_t * contact );
kf_status_t kf_shape_manifold( const kf_shape_t * left, kf_pose_t left_pose, const kf_shape_t * right, kf_pose_t right_pose, kf_fixed_t margin, kf_manifold_t * manifold );
kf_status_t kf_shape_sweep( const kf_shape_t * moving, kf_pose_t pose, kf_vec3_t displacement, const kf_shape_t * obstacle, kf_pose_t obstacle_pose, kf_sweep_hit_t * hit );
kf_status_t kf_shape_raycast( const kf_ray_t * ray, const kf_shape_t * shape, kf_pose_t pose, kf_sweep_hit_t * hit );

/* Integer seconds numerator/denominator per tick; defaults to 1/25. */
typedef struct kf_timestep_t
{
    uint32_t numerator;
    uint32_t denominator;
} kf_timestep_t;

kf_status_t kf_integrate_scalar( kf_fixed_t value, kf_timestep_t step, int64_t * remainder, kf_fixed_t * delta );
kf_status_t kf_checked_add( kf_fixed_t left, kf_fixed_t right, kf_fixed_t * result );
kf_status_t kf_checked_mul( kf_fixed_t left, kf_fixed_t right, kf_fixed_t * result );

typedef void * (*kf_allocate_fn)( void * context, size_t bytes );
typedef void (*kf_free_fn)( void * context, void * allocation );
typedef struct kf_allocator_t
{
    kf_allocate_fn allocate;
    kf_free_fn free;
    void * context;
} kf_allocator_t;

typedef struct kf_world_t kf_world_t;

typedef struct kf_world_config_t
{
    kf_timestep_t step;
    kf_vec3_t gravity;
    uint32_t solver_iterations; /* velocity iterations per substep */
    uint32_t ccd_iterations; /* continuous impacts per fast body and tick */
    uint32_t sleep_ticks;
    kf_fixed_t sleep_speed;
    kf_fixed_t contact_slop; /* allowed penetration; speculative distance is four slops */
    uint32_t substeps;
    kf_fixed_t restitution_threshold; /* slower approach speeds do not bounce */
    kf_fixed_t contact_hertz;
    kf_fixed_t contact_damping_ratio;
    kf_fixed_t contact_push_speed; /* maximum penetration recovery speed */
} kf_world_config_t;

typedef struct kf_capacity_t
{
    size_t objects;
    size_t shapes;
    size_t constraints;
    size_t contacts; /* shape pair manifolds per step; zero selects four per shape */
    size_t triggers; /* trigger overlaps after a step; zero selects one per shape */
} kf_capacity_t;

typedef enum kf_body_type_t
{
    KF_BODY_STATIC = 0,
    KF_BODY_KINEMATIC,
    KF_BODY_DYNAMIC
} kf_body_type_t;

/* Rotation about these world axes is locked; X and Z keep a body upright. */
#define KF_LOCK_ROTATION_X 1u
#define KF_LOCK_ROTATION_Y 2u
#define KF_LOCK_ROTATION_Z 4u

typedef struct kf_body_desc_t
{
    uint32_t id; /* nonzero application-assigned stable id */
    kf_body_type_t type;
    kf_pose_t pose;
    kf_vec3_t velocity; /* center of mass */
    kf_vec3_t angular_velocity; /* radians/second, world axes */
    kf_fixed_t mass;
    kf_fixed_t friction;
    kf_fixed_t restitution;
    kf_fixed_t linear_damping;
    kf_fixed_t angular_damping;
    kf_fixed_t gravity_scale; /* multiplies world gravity; 1 by default */
    uint32_t rotation_locks; /* KF_LOCK_ROTATION_* */
    uint32_t category;
    uint32_t mask;
    kf_bool_t trigger;
} kf_body_desc_t;

typedef struct kf_shape_desc_t
{
    uint32_t id; /* nonzero, unique within the object */
    kf_shape_t shape; /* local to body pose */
} kf_shape_desc_t;

/* Remainders are in raw units of 1 / (step.denominator * substeps). */
typedef struct kf_body_state_t
{
    kf_handle_t handle;
    kf_pose_t pose;
    kf_vec3_t velocity;
    kf_vec3_t angular_velocity;
    kf_fixed_t inverse_mass;
    int64_t position_remainder[3];
    int64_t velocity_remainder[3];
    kf_pose_t target; /* kinematic pose reached by the next step */
    kf_bool_t has_target;
    uint32_t sleep_counter;
    kf_bool_t sleeping;
} kf_body_state_t;

kf_world_config_t kf_world_config_default( void );
kf_body_desc_t kf_body_desc_default( uint32_t id );
kf_status_t kf_world_create( const kf_world_config_t * config, const kf_allocator_t * allocator, kf_world_t ** world );
void kf_world_destroy( kf_world_t * world );
kf_status_t kf_world_reserve( kf_world_t * world, kf_capacity_t capacity );
kf_status_t kf_world_add_body( kf_world_t * world, const kf_body_desc_t * body, const kf_shape_desc_t * shapes, size_t count, kf_handle_t * handle );
kf_status_t kf_world_remove_body( kf_world_t * world, kf_handle_t handle );
kf_status_t kf_world_get_body( const kf_world_t * world, kf_handle_t handle, kf_body_state_t * state );
kf_status_t kf_world_set_body( kf_world_t * world, const kf_body_state_t * state );
kf_status_t kf_world_update_body( kf_world_t * world, kf_handle_t handle, const kf_body_desc_t * body );
/* Applies a descriptor's type, mass, material, damping, gravity scale, locks,
   filter and trigger flag while keeping the current pose and remainders. Motion
   is kept, except that a body made static stops. */
kf_status_t kf_world_modify_body( kf_world_t * world, kf_handle_t handle, const kf_body_desc_t * body );
kf_status_t kf_world_update_shape( kf_world_t * world, kf_handle_t handle, const kf_shape_desc_t * shape );
kf_status_t kf_world_apply_impulse( kf_world_t * world, kf_handle_t body, kf_vec3_t impulse );
kf_status_t kf_world_apply_impulse_at( kf_world_t * world, kf_handle_t body, kf_vec3_t impulse, kf_vec3_t point );
/* Replaces the velocities of a dynamic or kinematic body without moving it. */
kf_status_t kf_world_set_velocity( kf_world_t * world, kf_handle_t body, kf_vec3_t velocity, kf_vec3_t angular_velocity );
kf_status_t kf_world_set_kinematic_target( kf_world_t * world, kf_handle_t body, kf_pose_t target );
kf_status_t kf_world_wake_body( kf_world_t * world, kf_handle_t body );
kf_status_t kf_world_rebuild_index( kf_world_t * world );

typedef struct kf_query_filter_t
{
    uint32_t category;
    uint32_t mask;
    const kf_handle_t * exclude;
    size_t exclude_count;
    kf_bool_t include_triggers;
} kf_query_filter_t;

kf_query_filter_t kf_query_filter_default( void );
kf_status_t kf_world_sweep_first( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, kf_vec3_t displacement, const kf_query_filter_t * filter, kf_sweep_hit_t * hit );
kf_status_t kf_world_sweep_all( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, kf_vec3_t displacement, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required );
kf_status_t kf_world_raycast_first( kf_world_t * world, const kf_ray_t * ray, const kf_query_filter_t * filter, kf_sweep_hit_t * hit );
kf_status_t kf_world_raycast_all( kf_world_t * world, const kf_ray_t * ray, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required );
kf_status_t kf_world_contacts( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, const kf_query_filter_t * filter, kf_contact_t * contacts, size_t capacity, size_t * required );
kf_status_t kf_world_query_region( kf_world_t * world, kf_aabb_t region, const kf_query_filter_t * filter, kf_handle_t * objects, size_t capacity, size_t * required );
kf_status_t kf_world_depenetrate( kf_world_t * world, const kf_shape_t * shape, kf_pose_t * pose, const kf_query_filter_t * filter, uint32_t iterations, kf_fixed_t maximum_distance, kf_vec3_t * correction );

typedef enum kf_query_type_t { KF_QUERY_SWEEP, KF_QUERY_RAY, KF_QUERY_CONTACTS, KF_QUERY_REGION } kf_query_type_t;
typedef struct kf_query_t
{
    kf_query_type_t type;
    kf_shape_t shape;
    kf_pose_t pose;
    kf_vec3_t displacement;
    kf_ray_t ray;
    kf_aabb_t region;
    kf_query_filter_t filter;
    void * output;
    size_t capacity;
    size_t required;
    kf_status_t status;
} kf_query_t;
kf_status_t kf_world_query_batch( kf_world_t * world, kf_query_t * queries, size_t count );

/* Relative motion uses both complete displacements over the same time interval. */
kf_status_t kf_shape_sweep_relative( const kf_shape_t * moving, kf_pose_t pose, kf_vec3_t displacement, const kf_shape_t * target, kf_pose_t target_pose, kf_vec3_t target_displacement, kf_sweep_hit_t * hit );
kf_status_t kf_world_sweep_moving( kf_world_t * world, const kf_shape_t * moving, kf_pose_t pose, kf_vec3_t displacement, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required );
/* Quadratic Bezier center path. Segment limit errors are explicit, never silently coarsened. */
kf_status_t kf_world_sweep_curve( kf_world_t * world, const kf_shape_t * moving, kf_pose_t pose, kf_vec3_t control, kf_vec3_t end, kf_fixed_t tolerance, uint32_t maximum_segments, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required );

typedef enum kf_constraint_type_t { KF_CONSTRAINT_DISTANCE = 0, KF_CONSTRAINT_MAX_DISTANCE } kf_constraint_type_t;
typedef struct kf_constraint_desc_t
{
    uint32_t id;
    kf_constraint_type_t type;
    kf_handle_t left;
    kf_handle_t right;
    kf_vec3_t anchor_left; /* local anchors rotate with their bodies */
    kf_vec3_t anchor_right;
    kf_fixed_t length;
    kf_fixed_t hertz; /* spring frequency; zero keeps the link rigid */
    kf_fixed_t damping_ratio; /* spring damping, used with a nonzero frequency */
} kf_constraint_desc_t;

typedef struct kf_constraint_state_t
{
    kf_handle_t handle;
    kf_constraint_desc_t desc;
    kf_fixed_t impulse;
} kf_constraint_state_t;

kf_status_t kf_world_add_constraint( kf_world_t * world, const kf_constraint_desc_t * desc, kf_handle_t * handle );
kf_status_t kf_world_remove_constraint( kf_world_t * world, kf_handle_t handle );
kf_status_t kf_world_get_constraint( const kf_world_t * world, kf_handle_t handle, kf_constraint_state_t * state );
kf_status_t kf_world_set_constraint( kf_world_t * world, const kf_constraint_state_t * state );
kf_status_t kf_world_step( kf_world_t * world );
uint64_t kf_world_tick( const kf_world_t * world );

/* A shape pair that touched in the last step: its contact points ended within
   contact_slop or carried a normal impulse. */
typedef struct kf_contact_report_t
{
    kf_handle_t left; /* body of the shape earlier in canonical order */
    uint32_t left_shape;
    kf_handle_t right;
    uint32_t right_shape;
    kf_vec3_t normal; /* from right toward left */
    kf_vec3_t point; /* contact centroid on the right shape at the step start */
    kf_fixed_t approach_speed; /* closing speed along the normal before the step */
    kf_bool_t began; /* the pair did not touch in the previous step */
} kf_contact_report_t;

/* Reports of the last step in canonical pair order. Body, shape and velocity
   edits keep them; adding or removing a body, reserving and restoring clear them
   until the next step. Pairs that meet only through continuous collision within
   one step are not reported. */
kf_status_t kf_world_contact_reports( const kf_world_t * world, kf_contact_report_t * reports, size_t capacity, size_t * required );

/* A trigger shape touching a shape that both filters accept, after a step. Pairs
   of two static bodies and of two triggers are not tracked. */
typedef struct kf_trigger_overlap_t
{
    kf_handle_t trigger;
    uint32_t trigger_shape;
    kf_handle_t visitor;
    uint32_t visitor_shape;
} kf_trigger_overlap_t;

typedef struct kf_trigger_event_t
{
    kf_trigger_overlap_t overlap;
    kf_bool_t began; /* otherwise the overlap ended */
} kf_trigger_event_t;

/* Overlaps that began or ended in the last step, in canonical pair order. They
   stay available until the next step; only a restore clears them. When a step
   finds more overlaps than reserved, it keeps the previous overlaps and this
   returns KF_CAPACITY_EXCEEDED with the overlap count to reserve. */
kf_status_t kf_world_trigger_events( const kf_world_t * world, kf_trigger_event_t * events, size_t capacity, size_t * required );

/* Snapshot data, not a file/network format. Includes inactive generations and all remainders. */
typedef struct kf_object_snapshot_t
{
    kf_body_desc_t desc;
    kf_body_state_t state;
    kf_bool_t active;
} kf_object_snapshot_t;
typedef struct kf_shape_snapshot_t
{
    uint32_t object_id;
    kf_shape_desc_t desc;
} kf_shape_snapshot_t;
typedef struct kf_constraint_snapshot_t
{
    kf_constraint_state_t state;
    kf_bool_t active;
} kf_constraint_snapshot_t;
typedef struct kf_contact_point_snapshot_t
{
    kf_vec3_t anchor; /* relative to the right body's center of mass, in its frame */
    int64_t normal_impulse;
    int64_t tangent_impulse[3]; /* world axes */
} kf_contact_point_snapshot_t;
/* Contact impulses carried to the next step, keyed by the canonical shape pair. */
typedef struct kf_contact_snapshot_t
{
    uint32_t left_object;
    uint32_t left_shape;
    uint32_t right_object;
    uint32_t right_shape;
    uint32_t count;
    kf_bool_t touching; /* the pair touched in the step that carried it */
    kf_contact_point_snapshot_t points[KF_MANIFOLD_MAX_POINTS];
} kf_contact_snapshot_t;
typedef struct kf_snapshot_t
{
    uint32_t version;
    kf_world_config_t config;
    uint64_t tick;
    kf_object_snapshot_t * objects;
    size_t object_capacity;
    size_t object_count;
    kf_shape_snapshot_t * shapes;
    size_t shape_capacity;
    size_t shape_count;
    kf_constraint_snapshot_t * constraints;
    size_t constraint_capacity;
    size_t constraint_count;
    kf_contact_snapshot_t * contacts;
    size_t contact_capacity;
    size_t contact_count;
    kf_trigger_overlap_t * triggers;
    size_t trigger_capacity;
    size_t trigger_count;
} kf_snapshot_t;
kf_status_t kf_world_save( const kf_world_t * world, kf_snapshot_t * snapshot );
kf_status_t kf_world_restore( kf_world_t * world, const kf_snapshot_t * snapshot );

typedef struct kf_character_state_t
{
    kf_vec3_t position;
    kf_vec3_t velocity;
    int64_t position_remainder[3];
    int64_t gravity_remainder;
    kf_bool_t grounded;
    kf_handle_t support;
    uint32_t support_shape_id;
    uint64_t support_shape_signature;
    kf_vec3_t support_normal;
    kf_vec3_t support_velocity;
    kf_pose_t support_pose;
} kf_character_state_t;

typedef struct kf_character_settings_t
{
    kf_fixed_t radius;
    kf_fixed_t height;
    kf_fixed_t step_height;
    kf_fixed_t snap_distance;
    kf_fixed_t minimum_ground_normal_y; /* cosine of maximum slope */
    kf_fixed_t skin;
    kf_fixed_t gravity;
    uint32_t slide_iterations;
    uint32_t depenetration_iterations;
    kf_fixed_t maximum_depenetration;
    kf_bool_t constrain_to_region;
    kf_aabb_t region;
    kf_fixed_t ledge_drop; /* nonzero: a grounded move stops instead of leaving no
                              walkable floor within this drop below the center */
} kf_character_settings_t;

typedef struct kf_character_move_result_t
{
    kf_vec3_t displacement;
    kf_bool_t stepped;
    kf_bool_t landed;
    kf_bool_t hit_ceiling;
    kf_bool_t held_at_ledge;
    kf_fixed_t landing_speed; /* downward speed when the move landed */
    size_t contact_count;
} kf_character_move_result_t;

kf_character_settings_t kf_character_settings_default( void );
kf_status_t kf_character_move( kf_world_t * world, kf_character_state_t * state, const kf_character_settings_t * settings, const kf_query_filter_t * filter, kf_contact_t * contacts, size_t capacity, kf_character_move_result_t * result );

#endif
