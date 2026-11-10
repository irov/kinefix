#include "kinefix_internal.h"

#include <limits.h>
#include <string.h>

#define KF_DYNAMICS_ONE INT64_C(65536)
#define KF_DYNAMICS_TWO_PI INT64_C(411775)
#define KF_DYNAMICS_JOINT_DAMPING INT64_C(131072)
/* Inertia floor per unit mass, (1/64 unit)^2 in Q8.24. */
#define KF_DYNAMICS_INERTIA_FLOOR INT64_C(4096)
/* 2 * sqrt(2) in Q16.16; see kf_internal_quat_chord. */
#define KF_DYNAMICS_CHORD_SPEED INT64_C(185364)
#define KF_DYNAMICS_RELAX_ITERATIONS 1u
#define KF_DYNAMICS_RESPONSE_ITERATIONS 8u
#define KF_DYNAMICS_NO_EVENT UINT64_MAX
#define KF_DYNAMICS_MAX_SUBSTEPS 16u
/* Solver-only headroom, 1024 times the public per-axis speed limit. */
#define KF_DYNAMICS_WORKING_SPEED (INT64_C(1) << 36)
/* Carried point impulses match new points within this body-frame distance. */
#define KF_DYNAMICS_MATCH_DISTANCE (KF_FIXED_SCALE / 20)

#define KF_SOLVER_DYNAMIC 1u
#define KF_SOLVER_MOVING 2u
#define KF_SOLVER_WAKE 4u
#define KF_SOLVER_AWAKE_ISLAND 8u
#define KF_SOLVER_FAST 16u
#define KF_SOLVER_WOKEN 32u
#define KF_SOLVER_CONTINUOUS 64u
#define KF_SOLVER_INCOMING 128u

typedef struct kf_dynamics_segment_state_t
{
    int64_t velocity[3];
    int64_t angular[3];
    int64_t position_remainder[3];
    int64_t velocity_remainder[3];
} kf_dynamics_segment_state_t;

/* Substep history of one body during a step. It is kept apart from the hot
   solver body data and is never cleared: rotation_count marks its valid prefix. */
struct kf_solver_path_t
{
    kf_rotation_segment_t segments[KF_DYNAMICS_MAX_SUBSTEPS];
    kf_dynamics_segment_state_t states[KF_DYNAMICS_MAX_SUBSTEPS];
};

typedef struct kf_solver_point_t
{
    kf_vec3_t anchor_a; /* contact point on A relative to its center at step start */
    kf_vec3_t anchor_b;
    int64_t base_separation;
    int64_t normal_mass; /* Q32.32 */
    int64_t tangent_mass[2];
    int64_t normal_impulse;
    int64_t tangent_impulse[2];
    int64_t maximum_impulse;
    int64_t relative_velocity;
} kf_solver_point_t;

struct kf_solver_body_t
{
    int64_t velocity[3];
    int64_t angular[3];
    int64_t gravity[3]; /* world gravity times the body's gravity scale */
    int64_t inverse_mass;
    kf_tensor_t inverse_inertia; /* world axes at step start, Q32.32 */
    kf_mass_t mass;
    kf_vec3_t center;
    kf_vec3_t start_center;
    kf_quat_t rotation;
    kf_quat_t start_rotation;
    kf_matrix_t delta; /* rotation since step start */
    kf_vec3_t delta_position;
    int64_t position_remainder[3];
    int64_t velocity_remainder[3];
    /* Continuous path over the tick interval [path_start, 1] for fast bodies. */
    uint64_t path_start;
    kf_vec3_t path_center;
    kf_quat_t path_rotation;
    kf_vec3_t end_center;
    kf_quat_t end_rotation;
    uint32_t rotation_count; /* recorded segments in the body's solver path */
    int64_t rotation_distance; /* Q30 actual angular travel, including full turns */
    kf_aabb_t sweep; /* bounds of the whole current path */
    uint64_t event_time;
    size_t event_shape;
    size_t event_other_shape;
    size_t event_other;
    uint32_t events;
    uint32_t island_counter;
    size_t island;
    uint32_t flags;
};

/* A is the body of the right shape, B the left one; the normal points from A to B. */
struct kf_solver_manifold_t
{
    size_t body_a;
    size_t body_b;
    kf_contact_snapshot_t carried; /* key and impulses carried to the next step */
    kf_vec3_t normal;
    kf_vec3_t tangents[2];
    int64_t friction;
    int64_t restitution;
    kf_bool_t fixed; /* touches a static body */
    kf_bool_t began; /* touching now but not in the previous step; until solved, only the latter */
    uint32_t count;
    kf_solver_point_t points[KF_MANIFOLD_MAX_POINTS];
};

typedef struct kf_dynamics_soft_t
{
    int64_t bias_rate;
    int64_t mass_scale;
    int64_t impulse_scale;
} kf_dynamics_soft_t;

/* A is the right body, B the left one; the axis points from A's anchor to B's. */
struct kf_solver_joint_t
{
    size_t body_a;
    size_t body_b;
    kf_vec3_t anchor_a;
    kf_vec3_t anchor_b;
    kf_vec3_t axis;
    int64_t error;
    int64_t mass;
    int64_t impulse;
    kf_dynamics_soft_t spring;
    kf_bool_t active;
    kf_bool_t limit;
    kf_bool_t elastic; /* a spring link instead of a rigid one */
};

typedef struct kf_dynamics_context_t
{
    int64_t numerator; /* substep seconds as numerator / denominator */
    int64_t denominator;
    int64_t step;
    int64_t inverse_step;
    kf_dynamics_soft_t contact;
    kf_dynamics_soft_t fixed; /* contacts with static bodies, twice as stiff */
    kf_dynamics_soft_t joint;
    int64_t push_speed;
    int64_t slop;
    int64_t speculative;
} kf_dynamics_context_t;

size_t kf_internal_solver_body_size( void )
{
    return sizeof( kf_solver_body_t );
}

size_t kf_internal_solver_manifold_size( void )
{
    return sizeof( kf_solver_manifold_t );
}

size_t kf_internal_solver_path_size( void )
{
    return sizeof( kf_solver_path_t );
}

static kf_solver_path_t * kf_dynamics_path( const kf_world_t * world, const kf_solver_body_t * body )
{
    return world->paths + (body - world->bodies);
}

size_t kf_internal_solver_joint_size( void )
{
    return sizeof( kf_solver_joint_t );
}

static kf_fixed_t kf_dynamics_narrow( int64_t value )
{
    if( value > INT32_MAX )
    {
        return INT32_MAX;
    }
    if( value < INT32_MIN )
    {
        return INT32_MIN;
    }
    return (kf_fixed_t)value;
}

static kf_bool_t kf_dynamics_vector_valid( kf_vec3_t value, kf_fixed_t limit )
{
    if( value.x < -limit || value.x > limit )
    {
        return KF_FALSE;
    }
    if( value.y < -limit || value.y > limit )
    {
        return KF_FALSE;
    }
    if( value.z < -limit || value.z > limit )
    {
        return KF_FALSE;
    }
    return KF_TRUE;
}

/* Length of a Q16.16 vector with 64-bit components. */
static int64_t kf_dynamics_length( const int64_t vector[3] )
{
    uint64_t maximum = 0;
    uint64_t square = 0;
    uint32_t shift = 0;
    uint32_t axis;
    for( axis = 0; axis != 3u; ++axis )
    {
        if( kf_internal_magnitude( vector[axis] ) > maximum )
        {
            maximum = kf_internal_magnitude( vector[axis] );
        }
    }
    while( (maximum >> shift) >= (UINT64_C(1) << 31) )
    {
        ++shift;
    }
    for( axis = 0; axis != 3u; ++axis )
    {
        uint64_t component = kf_internal_magnitude( vector[axis] ) >> shift;
        square += component * component;
    }
    {
        uint64_t length = kf_internal_sqrt( square ) << shift;
        return length > (uint64_t)INT64_MAX ? INT64_MAX : (int64_t)length;
    }
}

/* Scales a vector uniformly so that no component exceeds the limit. */
static void kf_dynamics_limit( int64_t vector[3], int64_t limit )
{
    uint64_t maximum = 0;
    uint32_t axis;
    for( axis = 0; axis != 3u; ++axis )
    {
        if( kf_internal_magnitude( vector[axis] ) > maximum )
        {
            maximum = kf_internal_magnitude( vector[axis] );
        }
    }
    if( maximum <= (uint64_t)limit )
    {
        return;
    }
    for( axis = 0; axis != 3u; ++axis )
    {
        vector[axis] = kf_internal_mul_div( vector[axis], limit, (int64_t)maximum, 0 );
    }
}

static int64_t kf_dynamics_rational( int64_t value, int64_t numerator, int64_t denominator, int64_t * remainder )
{
    int64_t total = value * numerator + *remainder;
    *remainder = total % denominator;
    return total / denominator;
}

static void kf_dynamics_cross( kf_vec3_t left, const int64_t right[3], int64_t result[3] )
{
    result[0] = kf_internal_mul_shift_term( left.y, right[2], 16 ) - kf_internal_mul_shift_term( left.z, right[1], 16 );
    result[1] = kf_internal_mul_shift_term( left.z, right[0], 16 ) - kf_internal_mul_shift_term( left.x, right[2], 16 );
    result[2] = kf_internal_mul_shift_term( left.x, right[1], 16 ) - kf_internal_mul_shift_term( left.y, right[0], 16 );
}

static void kf_dynamics_point_velocity( const kf_solver_body_t * body, kf_vec3_t anchor, int64_t result[3] )
{
    int64_t cross[3];
    kf_dynamics_cross( anchor, body->angular, cross );
    result[0] = body->velocity[0] - cross[0];
    result[1] = body->velocity[1] - cross[1];
    result[2] = body->velocity[2] - cross[2];
}

/* Relative velocity of B's anchor with respect to A's anchor along a direction. */
static int64_t kf_dynamics_relative_velocity( const kf_solver_body_t * a, const kf_solver_body_t * b, kf_vec3_t anchor_a, kf_vec3_t anchor_b, kf_vec3_t direction )
{
    int64_t velocity_a[3];
    int64_t velocity_b[3];
    kf_dynamics_point_velocity( a, anchor_a, velocity_a );
    kf_dynamics_point_velocity( b, anchor_b, velocity_b );
    return kf_internal_mul_shift( velocity_b[0] - velocity_a[0], direction.x, 16 ) + kf_internal_mul_shift( velocity_b[1] - velocity_a[1], direction.y, 16 ) + kf_internal_mul_shift( velocity_b[2] - velocity_a[2], direction.z, 16 );
}

static int64_t kf_dynamics_working_velocity( int64_t value )
{
    if( value < -KF_DYNAMICS_WORKING_SPEED )
    {
        return -KF_DYNAMICS_WORKING_SPEED;
    }
    if( value > KF_DYNAMICS_WORKING_SPEED )
    {
        return KF_DYNAMICS_WORKING_SPEED;
    }
    return value;
}

static void kf_dynamics_apply( kf_solver_body_t * body, kf_vec3_t anchor, kf_vec3_t direction, int64_t impulse )
{
    int64_t linear[3];
    int64_t torque[3];
    int64_t delta[3];
    uint32_t axis;
    if( (body->flags & KF_SOLVER_DYNAMIC) == 0 || impulse == 0 )
    {
        return;
    }
    linear[0] = kf_internal_mul_shift( impulse, direction.x, 16 );
    linear[1] = kf_internal_mul_shift( impulse, direction.y, 16 );
    linear[2] = kf_internal_mul_shift( impulse, direction.z, 16 );
    kf_dynamics_cross( anchor, linear, torque );
    kf_internal_tensor_apply( &body->inverse_inertia, torque, delta );
    /* Velocities stay within the working bound and each delta within three
       bounded terms, so the sums cannot overflow before they are clamped. */
    for( axis = 0; axis != 3u; ++axis )
    {
        body->velocity[axis] = kf_dynamics_working_velocity( body->velocity[axis] + kf_internal_mul_shift_term( body->inverse_mass, linear[axis], 16 ) );
        body->angular[axis] = kf_dynamics_working_velocity( body->angular[axis] + delta[axis] );
    }
}

static int64_t kf_dynamics_angular_mass( const kf_solver_body_t * body, kf_vec3_t anchor, kf_vec3_t direction )
{
    int64_t axis[3] = {direction.x, direction.y, direction.z};
    int64_t lever[3];
    int64_t rotated[3];
    if( (body->flags & KF_SOLVER_DYNAMIC) == 0 )
    {
        return 0;
    }
    kf_dynamics_cross( anchor, axis, lever );
    kf_internal_tensor_apply( &body->inverse_inertia, lever, rotated );
    return body->inverse_mass + kf_internal_mul_shift( lever[0], rotated[0], 16 ) + kf_internal_mul_shift( lever[1], rotated[1], 16 ) + kf_internal_mul_shift( lever[2], rotated[2], 16 );
}

/* Q32.32 reciprocal of the effective inverse mass along a direction. */
static int64_t kf_dynamics_effective_mass( const kf_solver_body_t * a, const kf_solver_body_t * b, kf_vec3_t anchor_a, kf_vec3_t anchor_b, kf_vec3_t direction )
{
    int64_t inverse = kf_dynamics_angular_mass( a, anchor_a, direction ) + kf_dynamics_angular_mass( b, anchor_b, direction );
    if( inverse <= 0 )
    {
        return 0;
    }
    return kf_internal_div_shift( 1, inverse, 48 );
}

static void kf_dynamics_clamp_body( kf_solver_body_t * body )
{
    kf_dynamics_limit( body->velocity, KF_SPEED_LIMIT );
    kf_dynamics_limit( body->angular, KF_SPEED_LIMIT );
}

/* A conservative sphere proves containment away from the world boundary. */
static kf_bool_t kf_dynamics_center_interior( const kf_solver_body_t * body )
{
    int64_t limit = (int64_t)KF_COORDINATE_LIMIT - body->mass.extent - KF_GEOMETRY_TOLERANCE;
    if( limit < 0 )
    {
        return KF_FALSE;
    }
    return kf_dynamics_vector_valid( body->center, (kf_fixed_t)limit );
}

/* Origin intervals admitted by the actual rotated shapes, including the origin
   itself. Use the same stored quaternion representation as the final commit. */
static kf_bool_t kf_dynamics_containment_range( const kf_world_t * world, size_t body_index, kf_vec3_t * offset, int64_t low[3], int64_t high[3] )
{
    const kf_solver_body_t * body = world->bodies + body_index;
    const kf_object_slot_t * object = world->objects + body_index;
    kf_pose_t pose = {0};
    kf_matrix_t matrix;
    size_t begin;
    size_t end;
    uint32_t axis;
    pose.rotation = object->state.pose.rotation;
    if( body->rotation.x != body->start_rotation.x || body->rotation.y != body->start_rotation.y || body->rotation.z != body->start_rotation.z || body->rotation.w != body->start_rotation.w )
    {
        pose.rotation = kf_internal_rotation( body->rotation );
    }
    matrix = kf_internal_matrix( kf_internal_quat( pose.rotation ) );
    *offset = kf_internal_matrix_apply( &matrix, body->mass.center );
    for( axis = 0; axis != 3u; ++axis )
    {
        low[axis] = -KF_COORDINATE_LIMIT;
        high[axis] = KF_COORDINATE_LIMIT;
    }
    if( kf_dynamics_center_interior( body ) == KF_TRUE &&
        kf_dynamics_vector_valid( kf_vec3_sub( body->center, *offset ), KF_COORDINATE_LIMIT ) == KF_TRUE )
    {
        return KF_TRUE;
    }
    kf_internal_shape_range( world, object->desc.id, &begin, &end );
    for( ; begin != end; ++begin )
    {
        kf_aabb_t bounds;
        int64_t minimum[3];
        int64_t maximum[3];
        if( kf_internal_shape_unbounded_bounds( &world->shapes[begin].desc.shape, pose, &bounds ) != KF_OK )
        {
            return KF_FALSE;
        }
        minimum[0] = bounds.minimum.x;
        minimum[1] = bounds.minimum.y;
        minimum[2] = bounds.minimum.z;
        maximum[0] = bounds.maximum.x;
        maximum[1] = bounds.maximum.y;
        maximum[2] = bounds.maximum.z;
        for( axis = 0; axis != 3u; ++axis )
        {
            int64_t shape_low = -(int64_t)KF_COORDINATE_LIMIT - minimum[axis];
            int64_t shape_high = (int64_t)KF_COORDINATE_LIMIT - maximum[axis];
            if( low[axis] < shape_low )
            {
                low[axis] = shape_low;
            }
            if( high[axis] > shape_high )
            {
                high[axis] = shape_high;
            }
            if( low[axis] > high[axis] )
            {
                return KF_FALSE;
            }
        }
    }
    return KF_TRUE;
}

/* Keeps both the stored origin and every shape point inside the coordinate
   range, preserving tangential motion and already admissible resting poses. */
static void kf_dynamics_contain( const kf_world_t * world, size_t body_index, kf_quat_t previous_rotation )
{
    kf_solver_body_t * body = world->bodies + body_index;
    int32_t * center[3] = {&body->center.x, &body->center.y, &body->center.z};
    int64_t low[3];
    int64_t high[3];
    kf_vec3_t offset;
    int64_t translation[3];
    uint32_t axis;
    /* The spherical bound is a cheap proof of safety: every shape point lies
       within the extent of the center of mass, and the stored origin within the
       length of the rotated center offset, which its L1 norm bounds. */
    if( kf_dynamics_center_interior( body ) == KF_TRUE )
    {
        int64_t offset = (int64_t)kf_internal_magnitude( body->mass.center.x ) + (int64_t)kf_internal_magnitude( body->mass.center.y ) + (int64_t)kf_internal_magnitude( body->mass.center.z );
        if( offset == 0 || kf_dynamics_vector_valid( body->center, (kf_fixed_t)( (int64_t)KF_COORDINATE_LIMIT - offset - KF_GEOMETRY_TOLERANCE ) ) == KF_TRUE )
        {
            return;
        }
    }
    if( kf_dynamics_containment_range( world, body_index, &offset, low, high ) == KF_FALSE )
    {
        /* A wide compound can rotate into an orientation that cannot fit at any
           translation. Keep the preceding admissible orientation instead. */
        body->rotation = previous_rotation;
        body->angular[0] = 0;
        body->angular[1] = 0;
        body->angular[2] = 0;
        (void)kf_dynamics_containment_range( world, body_index, &offset, low, high );
        body->delta = kf_internal_matrix( kf_internal_quat_multiply( body->rotation, kf_internal_quat_conjugate( body->start_rotation ) ) );
    }
    translation[0] = offset.x;
    translation[1] = offset.y;
    translation[2] = offset.z;
    for( axis = 0; axis != 3u; ++axis )
    {
        int64_t origin = (int64_t)*center[axis] - translation[axis];
        if( origin < low[axis] )
        {
            *center[axis] = kf_dynamics_narrow( low[axis] + translation[axis] );
            body->position_remainder[axis] = 0;
            if( body->velocity[axis] < 0 )
            {
                body->velocity[axis] = 0;
            }
        }
        if( origin > high[axis] )
        {
            *center[axis] = kf_dynamics_narrow( high[axis] + translation[axis] );
            body->position_remainder[axis] = 0;
            if( body->velocity[axis] > 0 )
            {
                body->velocity[axis] = 0;
            }
        }
    }
}

/* Locked world rotation axes get no angular response and no angular velocity. */
static void kf_dynamics_lock( uint32_t locks, kf_tensor_t * inverse_inertia, int64_t angular[3] )
{
    if( (locks & KF_LOCK_ROTATION_X) != 0 )
    {
        inverse_inertia->xx = 0;
        inverse_inertia->xy = 0;
        inverse_inertia->xz = 0;
        angular[0] = 0;
    }
    if( (locks & KF_LOCK_ROTATION_Y) != 0 )
    {
        inverse_inertia->yy = 0;
        inverse_inertia->xy = 0;
        inverse_inertia->yz = 0;
        angular[1] = 0;
    }
    if( (locks & KF_LOCK_ROTATION_Z) != 0 )
    {
        inverse_inertia->zz = 0;
        inverse_inertia->xz = 0;
        inverse_inertia->yz = 0;
        angular[2] = 0;
    }
}

static void kf_dynamics_wake( kf_object_slot_t * object )
{
    object->state.sleeping = KF_FALSE;
    object->state.sleep_counter = 0;
}

/* Signed first and second moments need the full unsigned 128-bit magnitude.
   Shape ids limit a body to fewer than 2^32 shapes; each volume is at most
   2^46 raw units, so total volume is below 2^78. Centroids span at most 8192
   units in X/Z and 8448 in Y. The weighted variance bound, including intrinsic
   Q8.24 inertia, keeps every second-moment sum below 2^128. */
typedef struct kf_dynamics_moment_t
{
    kf_wide_t magnitude;
    kf_bool_t negative;
} kf_dynamics_moment_t;

static void kf_dynamics_add_moment( kf_dynamics_moment_t * _moment, int64_t _volume, int64_t _value )
{
    kf_internal_wide_accumulate( &_moment->magnitude, &_moment->negative, kf_internal_wide_multiply( (uint64_t)_volume, kf_internal_magnitude( _value ) ), (kf_bool_t)(_value < 0) );
}

static int64_t kf_dynamics_average_moment( const kf_dynamics_moment_t * _moment, kf_wide_t _volume )
{
    uint64_t magnitude = kf_internal_wide_quotient( _moment->magnitude, _volume );
    int64_t average = kf_internal_signed( magnitude, _moment->negative );
    return average;
}

kf_status_t kf_internal_body_mass( const kf_world_t * world, const kf_object_slot_t * object, kf_mass_t * mass )
{
    size_t begin;
    size_t end;
    size_t index;
    kf_wide_t total = kf_internal_wide_multiply( 0, 0 );
    kf_dynamics_moment_t center[3];
    kf_dynamics_moment_t tensor[6];
    kf_tensor_t inertia = {0, 0, 0, 0, 0, 0};
    memset( mass, 0, sizeof(*mass) );
    memset( center, 0, sizeof(center) );
    memset( tensor, 0, sizeof(tensor) );
    mass->inverse_mass = object->desc.type == KF_BODY_DYNAMIC ? object->state.inverse_mass : 0;
    mass->thickness = KF_SIZE_LIMIT;
    kf_internal_shape_range( world, object->desc.id, &begin, &end );
    /* Volume weights; degenerate volumes keep a minimal share. */
    for( index = begin; index != end; ++index )
    {
        int64_t volume;
        kf_vec3_t centroid;
        kf_tensor_t shape_inertia;
        kf_fixed_t thickness;
        kf_status_t status = kf_internal_shape_mass( &world->shapes[index].desc.shape, &volume, &centroid, &shape_inertia, &thickness );
        if( status != KF_OK )
        {
            return status;
        }
        if( world->shapes[index].desc.shape.type == KF_SHAPE_TRIANGLE )
        {
            continue;
        }
        volume = volume < 1 ? 1 : volume;
        total = kf_internal_wide_add( total, kf_internal_wide_multiply( (uint64_t)volume, 1 ) );
        kf_dynamics_add_moment( center + 0, volume, centroid.x );
        kf_dynamics_add_moment( center + 1, volume, centroid.y );
        kf_dynamics_add_moment( center + 2, volume, centroid.z );
        if( thickness < mass->thickness )
        {
            mass->thickness = thickness;
        }
    }
    if( kf_internal_wide_compare( total, kf_internal_wide_multiply( 0, 0 ) ) != 0 )
    {
        /* Normalize only the complete sums. Per-child division loses small
           volume shares and introduces a shape-count-dependent centroid bias. */
        mass->center = (kf_vec3_t){(kf_fixed_t)kf_dynamics_average_moment( center + 0, total ), (kf_fixed_t)kf_dynamics_average_moment( center + 1, total ), (kf_fixed_t)kf_dynamics_average_moment( center + 2, total )};
        for( index = begin; index != end; ++index )
        {
            int64_t volume;
            kf_vec3_t centroid;
            kf_tensor_t shape_inertia;
            kf_fixed_t thickness;
            int64_t x;
            int64_t y;
            int64_t z;
            (void)kf_internal_shape_mass( &world->shapes[index].desc.shape, &volume, &centroid, &shape_inertia, &thickness );
            if( world->shapes[index].desc.shape.type == KF_SHAPE_TRIANGLE )
            {
                continue;
            }
            volume = volume < 1 ? 1 : volume;
            x = (int64_t)centroid.x - mass->center.x;
            y = (int64_t)centroid.y - mass->center.y;
            z = (int64_t)centroid.z - mass->center.z;
            /* Parallel axis offsets in Q8.24. */
            shape_inertia.xx += kf_internal_mul_shift( y, y, 8 ) + kf_internal_mul_shift( z, z, 8 );
            shape_inertia.yy += kf_internal_mul_shift( x, x, 8 ) + kf_internal_mul_shift( z, z, 8 );
            shape_inertia.zz += kf_internal_mul_shift( x, x, 8 ) + kf_internal_mul_shift( y, y, 8 );
            shape_inertia.xy -= kf_internal_mul_shift( x, y, 8 );
            shape_inertia.xz -= kf_internal_mul_shift( x, z, 8 );
            shape_inertia.yz -= kf_internal_mul_shift( y, z, 8 );
            kf_dynamics_add_moment( tensor + 0, volume, shape_inertia.xx );
            kf_dynamics_add_moment( tensor + 1, volume, shape_inertia.yy );
            kf_dynamics_add_moment( tensor + 2, volume, shape_inertia.zz );
            kf_dynamics_add_moment( tensor + 3, volume, shape_inertia.xy );
            kf_dynamics_add_moment( tensor + 4, volume, shape_inertia.xz );
            kf_dynamics_add_moment( tensor + 5, volume, shape_inertia.yz );
        }
        inertia.xx = kf_dynamics_average_moment( tensor + 0, total );
        inertia.yy = kf_dynamics_average_moment( tensor + 1, total );
        inertia.zz = kf_dynamics_average_moment( tensor + 2, total );
        inertia.xy = kf_dynamics_average_moment( tensor + 3, total );
        inertia.xz = kf_dynamics_average_moment( tensor + 4, total );
        inertia.yz = kf_dynamics_average_moment( tensor + 5, total );
    }
    for( index = begin; index != end; ++index )
    {
        kf_fixed_t extent;
        kf_status_t status = kf_internal_shape_extent( &world->shapes[index].desc.shape, mass->center, &extent );
        if( status != KF_OK )
        {
            return status;
        }
        if( extent > mass->extent )
        {
            mass->extent = extent;
        }
    }
    if( mass->inverse_mass != 0 )
    {
        kf_tensor_t inverse;
        inertia.xx += KF_DYNAMICS_INERTIA_FLOOR;
        inertia.yy += KF_DYNAMICS_INERTIA_FLOOR;
        inertia.zz += KF_DYNAMICS_INERTIA_FLOOR;
        inverse = kf_internal_tensor_inverse( inertia );
        mass->inverse_inertia.xx = kf_internal_mul_shift( inverse.xx, mass->inverse_mass, 16 );
        mass->inverse_inertia.yy = kf_internal_mul_shift( inverse.yy, mass->inverse_mass, 16 );
        mass->inverse_inertia.zz = kf_internal_mul_shift( inverse.zz, mass->inverse_mass, 16 );
        mass->inverse_inertia.xy = kf_internal_mul_shift( inverse.xy, mass->inverse_mass, 16 );
        mass->inverse_inertia.xz = kf_internal_mul_shift( inverse.xz, mass->inverse_mass, 16 );
        mass->inverse_inertia.yz = kf_internal_mul_shift( inverse.yz, mass->inverse_mass, 16 );
    }
    return KF_OK;
}

kf_vec3_t kf_internal_body_center( const kf_object_slot_t * object, const kf_mass_t * mass )
{
    kf_vec3_t offset = kf_rotation_apply( object->state.pose.rotation, mass->center );
    return (kf_vec3_t){kf_dynamics_narrow( (int64_t)object->state.pose.position.x + offset.x ), kf_dynamics_narrow( (int64_t)object->state.pose.position.y + offset.y ), kf_dynamics_narrow( (int64_t)object->state.pose.position.z + offset.z )};
}

/* Applies a world impulse at a world point to a dynamic body's state. */
static kf_status_t kf_dynamics_impulse( kf_world_t * world, kf_object_slot_t * object, kf_vec3_t impulse, const kf_vec3_t * point )
{
    int64_t linear[3] = {impulse.x, impulse.y, impulse.z};
    int64_t velocity[3];
    int64_t angular[3] = {object->state.angular_velocity.x, object->state.angular_velocity.y, object->state.angular_velocity.z};
    uint32_t axis;
    velocity[0] = object->state.velocity.x + kf_internal_mul_shift( object->state.inverse_mass, linear[0], 16 );
    velocity[1] = object->state.velocity.y + kf_internal_mul_shift( object->state.inverse_mass, linear[1], 16 );
    velocity[2] = object->state.velocity.z + kf_internal_mul_shift( object->state.inverse_mass, linear[2], 16 );
    if( point != NULL )
    {
        kf_mass_t mass;
        kf_tensor_t inverse_inertia;
        kf_matrix_t matrix = kf_internal_matrix( kf_internal_quat( object->state.pose.rotation ) );
        kf_vec3_t center;
        kf_vec3_t lever;
        int64_t torque[3];
        int64_t delta[3];
        kf_status_t status = kf_internal_body_mass( world, object, &mass );
        if( status != KF_OK )
        {
            return status;
        }
        center = kf_internal_body_center( object, &mass );
        lever = (kf_vec3_t){kf_dynamics_narrow( (int64_t)point->x - center.x ), kf_dynamics_narrow( (int64_t)point->y - center.y ), kf_dynamics_narrow( (int64_t)point->z - center.z )};
        inverse_inertia = kf_internal_tensor_rotate( &matrix, mass.inverse_inertia );
        kf_dynamics_lock( object->desc.rotation_locks, &inverse_inertia, angular );
        kf_dynamics_cross( lever, linear, torque );
        kf_internal_tensor_apply( &inverse_inertia, torque, delta );
        for( axis = 0; axis != 3u; ++axis )
        {
            angular[axis] += delta[axis];
        }
    }
    for( axis = 0; axis != 3u; ++axis )
    {
        if( velocity[axis] < -KF_SPEED_LIMIT || velocity[axis] > KF_SPEED_LIMIT || angular[axis] < -KF_SPEED_LIMIT || angular[axis] > KF_SPEED_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }
    }
    object->state.velocity = (kf_vec3_t){(kf_fixed_t)velocity[0], (kf_fixed_t)velocity[1], (kf_fixed_t)velocity[2]};
    object->state.angular_velocity = (kf_vec3_t){(kf_fixed_t)angular[0], (kf_fixed_t)angular[1], (kf_fixed_t)angular[2]};
    kf_dynamics_wake( object );
    return KF_OK;
}

kf_status_t kf_world_apply_impulse( kf_world_t * world, kf_handle_t body, kf_vec3_t impulse )
{
    kf_object_slot_t * object;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, body );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( object->desc.type != KF_BODY_DYNAMIC )
    {
        return KF_INVALID_ARGUMENT;
    }
    return kf_dynamics_impulse( world, object, impulse, NULL );
}

kf_status_t kf_world_apply_impulse_at( kf_world_t * world, kf_handle_t body, kf_vec3_t impulse, kf_vec3_t point )
{
    kf_object_slot_t * object;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, body );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( object->desc.type != KF_BODY_DYNAMIC )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_dynamics_vector_valid( point, KF_COORDINATE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    return kf_dynamics_impulse( world, object, impulse, &point );
}

kf_status_t kf_world_set_velocity( kf_world_t * world, kf_handle_t body, kf_vec3_t velocity, kf_vec3_t angular_velocity )
{
    kf_object_slot_t * object;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, body );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( object->desc.type == KF_BODY_STATIC )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_dynamics_vector_valid( velocity, KF_SPEED_LIMIT ) == KF_FALSE || kf_dynamics_vector_valid( angular_velocity, KF_SPEED_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    object->state.velocity = velocity;
    object->state.angular_velocity = angular_velocity;
    /* The latest command decides a kinematic body's motion. */
    object->state.has_target = KF_FALSE;
    if( object->desc.type == KF_BODY_DYNAMIC )
    {
        kf_dynamics_wake( object );
    }
    return KF_OK;
}

kf_status_t kf_internal_validate_kinematic_target( const kf_world_config_t * config, const kf_body_state_t * state, const kf_mass_t * mass )
{
    kf_quat_t rotation;
    kf_quat_t target;
    kf_matrix_t matrix;
    kf_matrix_t target_matrix;
    kf_vec3_t offset;
    kf_vec3_t target_offset;
    int64_t displacement[3];
    int64_t vector[3];
    uint32_t axis;
    if( state->has_target == KF_FALSE )
    {
        return KF_OK;
    }
    rotation = kf_internal_quat( state->pose.rotation );
    target = kf_internal_quat( state->target.rotation );
    matrix = kf_internal_matrix( rotation );
    target_matrix = kf_internal_matrix( target );
    offset = kf_internal_matrix_apply( &matrix, mass->center );
    target_offset = kf_internal_matrix_apply( &target_matrix, mass->center );
    displacement[0] = ((int64_t)state->target.position.x + target_offset.x) - ((int64_t)state->pose.position.x + offset.x);
    displacement[1] = ((int64_t)state->target.position.y + target_offset.y) - ((int64_t)state->pose.position.y + offset.y);
    displacement[2] = ((int64_t)state->target.position.z + target_offset.z) - ((int64_t)state->pose.position.z + offset.z);
    kf_internal_quat_rotation_vector( kf_internal_quat_multiply( target, kf_internal_quat_conjugate( rotation ) ), vector );
    for( axis = 0; axis != 3u; ++axis )
    {
        int64_t velocity = displacement[axis] * config->step.denominator / config->step.numerator;
        int64_t angular = vector[axis] * config->step.denominator / config->step.numerator;
        if( velocity < -KF_SPEED_LIMIT || velocity > KF_SPEED_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }
        if( angular < -KF_SPEED_LIMIT || angular > KF_SPEED_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }
    }
    return KF_OK;
}

kf_status_t kf_world_set_kinematic_target( kf_world_t * world, kf_handle_t body, kf_pose_t target )
{
    kf_object_slot_t * object;
    size_t begin;
    size_t end;
    size_t index;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, body );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( object->desc.type != KF_BODY_KINEMATIC )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_dynamics_vector_valid( target.position, KF_COORDINATE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_rotation_valid( target.rotation ) == KF_FALSE )
    {
        return KF_INVALID_ARGUMENT;
    }
    kf_internal_shape_range( world, object->desc.id, &begin, &end );
    for( index = begin; index != end; ++index )
    {
        kf_aabb_t bounds;
        kf_status_t status = kf_shape_bounds( &world->shapes[index].desc.shape, target, &bounds );
        if( status != KF_OK )
        {
            return status;
        }
    }
    {
        kf_body_state_t state = object->state;
        kf_mass_t mass;
        kf_status_t status = kf_internal_body_mass( world, object, &mass );
        if( status != KF_OK )
        {
            return status;
        }
        state.target = target;
        state.has_target = KF_TRUE;
        status = kf_internal_validate_kinematic_target( &world->config, &state, &mass );
        if( status != KF_OK )
        {
            return status;
        }
    }
    object->state.target = target;
    object->state.has_target = KF_TRUE;
    return KF_OK;
}

kf_status_t kf_world_wake_body( kf_world_t * world, kf_handle_t body )
{
    kf_object_slot_t * object;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, body );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    kf_dynamics_wake( object );
    return KF_OK;
}

static kf_status_t kf_dynamics_validate_constraint( kf_world_t * world, const kf_constraint_desc_t * desc )
{
    if( desc == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( desc->id == 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( desc->type != KF_CONSTRAINT_DISTANCE && desc->type != KF_CONSTRAINT_MAX_DISTANCE )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( desc->left.id == desc->right.id )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_internal_object( world, desc->left ) == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( kf_internal_object( world, desc->right ) == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( desc->length < 0 || desc->length > KF_SIZE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->hertz < 0 || desc->hertz > KF_SPEED_LIMIT || desc->damping_ratio < 0 || desc->damping_ratio > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_dynamics_vector_valid( desc->anchor_left, KF_SIZE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_dynamics_vector_valid( desc->anchor_right, KF_SIZE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    return KF_OK;
}

static kf_constraint_slot_t * kf_dynamics_constraint( kf_world_t * world, kf_handle_t handle )
{
    size_t index;
    for( index = 0; index != world->constraint_count; ++index )
    {
        kf_constraint_slot_t * slot = world->constraints + index;
        if( slot->state.handle.id != handle.id )
        {
            continue;
        }
        if( slot->state.handle.generation != handle.generation )
        {
            return NULL;
        }
        if( slot->active == KF_FALSE )
        {
            return NULL;
        }
        return slot;
    }
    return NULL;
}

kf_status_t kf_world_add_constraint( kf_world_t * world, const kf_constraint_desc_t * desc, kf_handle_t * handle )
{
    kf_constraint_slot_t * slot;
    kf_status_t status;
    uint32_t generation = 1;
    size_t index = 0;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( handle == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_dynamics_validate_constraint( world, desc );
    if( status != KF_OK )
    {
        return status;
    }
    while( index != world->constraint_count && world->constraints[index].state.desc.id < desc->id )
    {
        ++index;
    }
    if( index != world->constraint_count && world->constraints[index].state.desc.id == desc->id )
    {
        slot = world->constraints + index;
        if( slot->active == KF_TRUE )
        {
            return KF_DUPLICATE_ID;
        }
        if( slot->state.handle.generation == UINT32_MAX )
        {
            return KF_OVERFLOW;
        }
        generation = slot->state.handle.generation + 1;
    }
    else
    {
        if( world->constraint_count == world->capacity.constraints )
        {
            return KF_CAPACITY_EXCEEDED;
        }
        memmove( world->constraints + index + 1, world->constraints + index, (world->constraint_count - index) * sizeof(*world->constraints) );
        ++world->constraint_count;
        slot = world->constraints + index;
    }
    memset( slot, 0, sizeof(*slot) );
    slot->active = KF_TRUE;
    slot->state.desc = *desc;
    slot->state.handle = (kf_handle_t){desc->id, generation};
    *handle = slot->state.handle;
    kf_dynamics_wake( kf_internal_object( world, desc->left ) );
    kf_dynamics_wake( kf_internal_object( world, desc->right ) );
    return KF_OK;
}

kf_status_t kf_world_remove_constraint( kf_world_t * world, kf_handle_t handle )
{
    kf_constraint_slot_t * slot;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    slot = kf_dynamics_constraint( world, handle );
    if( slot == NULL )
    {
        return KF_STALE_HANDLE;
    }
    slot->active = KF_FALSE;
    kf_dynamics_wake( kf_internal_object( world, slot->state.desc.left ) );
    kf_dynamics_wake( kf_internal_object( world, slot->state.desc.right ) );
    return KF_OK;
}

kf_status_t kf_world_get_constraint( const kf_world_t * world, kf_handle_t handle, kf_constraint_state_t * state )
{
    size_t index;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    for( index = 0; index != world->constraint_count; ++index )
    {
        const kf_constraint_slot_t * slot = world->constraints + index;
        if( slot->state.handle.id != handle.id )
        {
            continue;
        }
        if( slot->state.handle.generation != handle.generation )
        {
            return KF_STALE_HANDLE;
        }
        if( slot->active == KF_FALSE )
        {
            return KF_STALE_HANDLE;
        }
        *state = slot->state;
        return KF_OK;
    }
    return KF_STALE_HANDLE;
}

kf_status_t kf_world_set_constraint( kf_world_t * world, const kf_constraint_state_t * state )
{
    kf_constraint_slot_t * slot;
    kf_status_t status;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state->handle.id != state->desc.id )
    {
        return KF_INVALID_ARGUMENT;
    }
    slot = kf_dynamics_constraint( world, state->handle );
    if( slot == NULL )
    {
        return KF_STALE_HANDLE;
    }
    status = kf_dynamics_validate_constraint( world, &state->desc );
    if( status != KF_OK )
    {
        return status;
    }
    slot->state = *state;
    kf_dynamics_wake( kf_internal_object( world, state->desc.left ) );
    kf_dynamics_wake( kf_internal_object( world, state->desc.right ) );
    return KF_OK;
}

static kf_bool_t kf_dynamics_pair_enabled( const kf_object_slot_t * left, const kf_object_slot_t * right )
{
    if( left == right )
    {
        return KF_FALSE;
    }
    if( left->active == KF_FALSE || right->active == KF_FALSE )
    {
        return KF_FALSE;
    }
    if( left->desc.trigger == KF_TRUE || right->desc.trigger == KF_TRUE )
    {
        return KF_FALSE;
    }
    if( (left->desc.category & right->desc.mask) == 0 )
    {
        return KF_FALSE;
    }
    if( (right->desc.category & left->desc.mask) == 0 )
    {
        return KF_FALSE;
    }
    if( left->desc.type != KF_BODY_DYNAMIC && right->desc.type != KF_BODY_DYNAMIC )
    {
        return KF_FALSE;
    }
    return KF_TRUE;
}

static kf_dynamics_soft_t kf_dynamics_softness( int64_t hertz, int64_t damping, int64_t step )
{
    kf_dynamics_soft_t soft = {0, KF_DYNAMICS_ONE, 0};
    int64_t travel;
    int64_t first;
    int64_t second;
    if( hertz <= 0 )
    {
        return soft;
    }
    /* Each coefficient is one wide operation on the inputs: h omega with
       omega = 2 pi hertz, then 2 zeta + h omega, h omega (2 zeta + h omega),
       omega / first, second / (1 + second) and 1 / (1 + second). */
    travel = kf_internal_mul3_shift( step, KF_DYNAMICS_TWO_PI, hertz, 32 );
    first = 2 * damping + travel;
    second = kf_internal_mul_shift( travel, first, 16 );
    soft.bias_rate = first == 0 ? 0 : kf_internal_mul_div( KF_DYNAMICS_TWO_PI, hertz, first, 0 );
    soft.mass_scale = kf_internal_div_shift( second, KF_DYNAMICS_ONE + second, 16 );
    soft.impulse_scale = kf_internal_div_shift( 1, KF_DYNAMICS_ONE + second, 32 );
    return soft;
}

static kf_dynamics_context_t kf_dynamics_context( const kf_world_t * world )
{
    kf_dynamics_context_t context;
    int64_t hertz;
    memset( &context, 0, sizeof(context) );
    context.numerator = world->config.step.numerator;
    context.denominator = (int64_t)world->config.step.denominator * world->config.substeps;
    context.step = context.numerator * KF_DYNAMICS_ONE / context.denominator;
    context.inverse_step = context.denominator * KF_DYNAMICS_ONE / context.numerator;
    /* Contacts stay below a quarter of the substep rate. */
    hertz = world->config.contact_hertz;
    if( hertz > context.inverse_step / 4 )
    {
        hertz = context.inverse_step / 4;
    }
    context.contact = kf_dynamics_softness( hertz, world->config.contact_damping_ratio, context.step );
    context.fixed = kf_dynamics_softness( hertz * 2, world->config.contact_damping_ratio, context.step );
    context.joint = kf_dynamics_softness( hertz * 2, KF_DYNAMICS_JOINT_DAMPING, context.step );
    context.push_speed = world->config.contact_push_speed;
    context.slop = world->config.contact_slop;
    context.speculative = (int64_t)world->config.contact_slop * 4 + KF_GEOMETRY_TOLERANCE;
    return context;
}

static const kf_matrix_t kf_dynamics_identity = {{{1073741824, 0, 0}, {0, 1073741824, 0}, {0, 0, 1073741824}}};

/* Rotation bound per tick of any point at the extent: 2 * sqrt(2) * extent * chord. */
static int64_t kf_dynamics_rotation_bound( kf_quat_t from, kf_quat_t to, kf_fixed_t extent )
{
    int64_t chord = kf_internal_quat_chord( from, to );
    if( chord == 0 )
    {
        return 0;
    }
    return kf_internal_mul3_shift( chord, extent, KF_DYNAMICS_CHORD_SPEED, 46 ) + 1;
}

/* Mass properties depend only on descriptors and shapes; they are recomputed
   after structural changes. */
static kf_status_t kf_dynamics_refresh_masses( kf_world_t * world )
{
    size_t index;
    if( world->masses_dirty == KF_FALSE )
    {
        return KF_OK;
    }
    for( index = 0; index != world->object_count; ++index )
    {
        const kf_object_slot_t * object = world->objects + index;
        memset( world->masses + index, 0, sizeof(*world->masses) );
        if( object->active == KF_TRUE && object->desc.type != KF_BODY_STATIC )
        {
            kf_status_t status = kf_internal_body_mass( world, object, world->masses + index );
            if( status != KF_OK )
            {
                return status;
            }
        }
    }
    world->masses_dirty = KF_FALSE;
    return KF_OK;
}

static kf_status_t kf_dynamics_prepare( kf_world_t * world )
{
    int64_t numerator = world->config.step.numerator;
    int64_t denominator = world->config.step.denominator;
    size_t index;
    kf_status_t status = kf_dynamics_refresh_masses( world );
    if( status != KF_OK )
    {
        return status;
    }
    for( index = 0; index != world->object_count; ++index )
    {
        kf_object_slot_t * object = world->objects + index;
        kf_solver_body_t * body = world->bodies + index;
        kf_matrix_t matrix;
        kf_vec3_t offset;
        uint32_t axis;
        memset( body, 0, sizeof(*body) );
        body->island = index;
        body->rotation = kf_internal_quat( object->state.pose.rotation );
        body->start_rotation = body->rotation;
        body->delta = kf_dynamics_identity;
        body->center = object->state.pose.position;
        body->start_center = body->center;
        body->event_time = KF_DYNAMICS_NO_EVENT;
        if( object->active == KF_FALSE || object->desc.type == KF_BODY_STATIC )
        {
            continue;
        }
        body->mass = world->masses[index];
        matrix = kf_internal_matrix( body->rotation );
        offset = kf_internal_matrix_apply( &matrix, body->mass.center );
        body->center = (kf_vec3_t){kf_dynamics_narrow( (int64_t)object->state.pose.position.x + offset.x ), kf_dynamics_narrow( (int64_t)object->state.pose.position.y + offset.y ), kf_dynamics_narrow( (int64_t)object->state.pose.position.z + offset.z )};
        body->start_center = body->center;
        body->velocity[0] = object->state.velocity.x;
        body->velocity[1] = object->state.velocity.y;
        body->velocity[2] = object->state.velocity.z;
        body->angular[0] = object->state.angular_velocity.x;
        body->angular[1] = object->state.angular_velocity.y;
        body->angular[2] = object->state.angular_velocity.z;
        for( axis = 0; axis != 3u; ++axis )
        {
            body->position_remainder[axis] = object->state.position_remainder[axis];
            body->velocity_remainder[axis] = object->state.velocity_remainder[axis];
        }
        if( object->desc.type == KF_BODY_DYNAMIC )
        {
            body->inverse_mass = body->mass.inverse_mass;
            body->inverse_inertia = kf_internal_tensor_rotate( &matrix, body->mass.inverse_inertia );
            kf_dynamics_lock( object->desc.rotation_locks, &body->inverse_inertia, body->angular );
            body->gravity[0] = kf_internal_mul_shift( world->config.gravity.x, object->desc.gravity_scale, 16 );
            body->gravity[1] = kf_internal_mul_shift( world->config.gravity.y, object->desc.gravity_scale, 16 );
            body->gravity[2] = kf_internal_mul_shift( world->config.gravity.z, object->desc.gravity_scale, 16 );
            if( object->state.sleeping == KF_FALSE )
            {
                body->flags |= KF_SOLVER_DYNAMIC;
            }
            continue;
        }
        if( object->state.has_target == KF_TRUE )
        {
            /* Velocities that reach the target pose at the end of this step. */
            kf_status_t target_status = kf_internal_validate_kinematic_target( &world->config, &object->state, &body->mass );
            kf_quat_t target = kf_internal_quat( object->state.target.rotation );
            kf_matrix_t target_matrix = kf_internal_matrix( target );
            kf_vec3_t target_offset = kf_internal_matrix_apply( &target_matrix, body->mass.center );
            int64_t target_center[3] = {(int64_t)object->state.target.position.x + target_offset.x, (int64_t)object->state.target.position.y + target_offset.y, (int64_t)object->state.target.position.z + target_offset.z};
            int64_t rotation[3];
            int64_t center[3] = {body->center.x, body->center.y, body->center.z};
            if( target_status != KF_OK )
            {
                return target_status;
            }
            kf_internal_quat_rotation_vector( kf_internal_quat_multiply( target, kf_internal_quat_conjugate( body->rotation ) ), rotation );
            for( axis = 0; axis != 3u; ++axis )
            {
                body->velocity[axis] = (target_center[axis] - center[axis]) * denominator / numerator;
                body->angular[axis] = kf_internal_mul_shift( rotation[axis], denominator, 0 ) / numerator;
            }
            kf_dynamics_clamp_body( body );
        }
        for( axis = 0; axis != 3u; ++axis )
        {
            if( body->velocity[axis] != 0 || body->angular[axis] != 0 )
            {
                body->flags |= KF_SOLVER_MOVING;
            }
        }
        if( object->state.has_target == KF_TRUE )
        {
            body->flags |= KF_SOLVER_MOVING;
        }
    }
    return KF_OK;
}

static size_t kf_dynamics_find( kf_solver_body_t * bodies, size_t index )
{
    while( bodies[index].island != index )
    {
        bodies[index].island = bodies[bodies[index].island].island;
        index = bodies[index].island;
    }
    return index;
}

static void kf_dynamics_union( kf_solver_body_t * bodies, size_t left, size_t right )
{
    size_t left_root = kf_dynamics_find( bodies, left );
    size_t right_root = kf_dynamics_find( bodies, right );
    if( left_root < right_root )
    {
        bodies[right_root].island = left_root;
    }
    else if( right_root < left_root )
    {
        bodies[left_root].island = right_root;
    }
}

/* The tree over scratch bounds is step-local; the committed step rebuilds the
   public index from current bounds. */
static void kf_dynamics_build_tree( kf_world_t * world )
{
    size_t index;
    world->index_dirty = KF_TRUE;
    for( index = 0; index != world->shape_count; ++index )
    {
        world->leaves[index].shape_index = index;
        world->leaves[index].bounds = world->scratch_bounds[index];
    }
    kf_internal_sort_leaves( world->leaves, world->shape_count );
    kf_internal_build_tree( world );
}

/* Shapes of dynamic and moving kinematic bodies start broad-phase scans; static
   and resting kinematic shapes are only found as partners. */
static kf_bool_t kf_dynamics_scanner( const kf_world_t * world, size_t shape_index )
{
    size_t object_index = world->shape_objects[shape_index];
    return (kf_bool_t)(world->objects[object_index].desc.type == KF_BODY_DYNAMIC || (world->bodies[object_index].flags & KF_SOLVER_MOVING) != 0);
}

/* Partners of a scanning shape; each pair is reported by exactly one scanner. */
static size_t kf_dynamics_candidates( kf_world_t * world, size_t left_index )
{
    kf_aabb_t bounds = world->scratch_bounds[left_index];
    size_t stack[64];
    size_t depth = 0;
    size_t count = 0;
    if( world->node_count == 0 )
    {
        return 0;
    }
    stack[depth++] = 0;
    while( depth != 0 )
    {
        const kf_index_node_t * node = world->nodes + stack[--depth];
        size_t shape_index = node->shape_index;
        if( node->object_id == world->shapes[left_index].object_id )
        {
            continue;
        }
        if( kf_internal_bounds_overlap( bounds, node->bounds ) == KF_FALSE )
        {
            continue;
        }
        if( shape_index == SIZE_MAX )
        {
            stack[depth++] = node->right;
            stack[depth++] = node->left;
            continue;
        }
        if( world->shape_objects[shape_index] == world->shape_objects[left_index] )
        {
            continue;
        }
        if( shape_index < left_index && kf_dynamics_scanner( world, shape_index ) == KF_TRUE )
        {
            continue;
        }
        world->candidates[count++] = shape_index;
    }
    return count;
}

static kf_aabb_t kf_dynamics_expand( kf_aabb_t bounds, int64_t amount )
{
    bounds.minimum = (kf_vec3_t){kf_dynamics_narrow( bounds.minimum.x - amount ), kf_dynamics_narrow( bounds.minimum.y - amount ), kf_dynamics_narrow( bounds.minimum.z - amount )};
    bounds.maximum = (kf_vec3_t){kf_dynamics_narrow( bounds.maximum.x + amount ), kf_dynamics_narrow( bounds.maximum.y + amount ), kf_dynamics_narrow( bounds.maximum.z + amount )};
    return bounds;
}

/* Predicted translation over the tick and a bound on rotational point motion. */
static void kf_dynamics_predict( const kf_world_t * world, const kf_solver_body_t * body, int64_t displacement[3], int64_t * rotation )
{
    int64_t numerator = world->config.step.numerator;
    int64_t denominator = world->config.step.denominator;
    int64_t angular = 0;
    uint32_t axis;
    for( axis = 0; axis != 3u; ++axis )
    {
        displacement[axis] = body->velocity[axis] * numerator / denominator;
        angular += (int64_t)kf_internal_magnitude( body->angular[axis] );
    }
    *rotation = kf_internal_mul_shift( angular * numerator / denominator, body->mass.extent, 16 );
    if( *rotation > 2 * (int64_t)body->mass.extent )
    {
        *rotation = 2 * (int64_t)body->mass.extent;
    }
}

static int64_t kf_dynamics_motion_bound( const kf_world_t * world, const kf_solver_body_t * body )
{
    int64_t displacement[3];
    int64_t rotation;
    if( (body->flags & (KF_SOLVER_DYNAMIC | KF_SOLVER_MOVING)) == 0 )
    {
        return 0;
    }
    kf_dynamics_predict( world, body, displacement, &rotation );
    return (int64_t)kf_internal_magnitude( displacement[0] ) + (int64_t)kf_internal_magnitude( displacement[1] ) + (int64_t)kf_internal_magnitude( displacement[2] ) + rotation;
}

/* Speculative range of a pair with a kinematic body, whose prescribed motion
   over the tick is known in advance. */
static int64_t kf_dynamics_pair_margin( const kf_world_t * world, const kf_dynamics_context_t * context, size_t kinematic )
{
    int64_t margin = context->speculative + kf_dynamics_motion_bound( world, world->bodies + kinematic );
    return margin > KF_SIZE_LIMIT ? KF_SIZE_LIMIT : margin;
}

static void kf_dynamics_tangents( kf_vec3_t normal, kf_vec3_t tangents[2] )
{
    kf_vec3_t first;
    if( kf_fixed_abs( normal.x ) >= 37837 )
    {
        first = kf_vec3_normalize( (kf_vec3_t){normal.y, -normal.x, 0} );
    }
    else
    {
        first = kf_vec3_normalize( (kf_vec3_t){0, normal.z, -normal.y} );
    }
    tangents[0] = first;
    tangents[1] = kf_vec3_normalize( kf_vec3_cross( normal, first ) );
}

static const kf_contact_snapshot_t * kf_dynamics_cached( const kf_world_t * world, const kf_contact_snapshot_t * key )
{
    size_t low = 0;
    size_t high = world->cache_count;
    while( low != high )
    {
        size_t middle = low + (high - low) / 2u;
        if( kf_internal_contact_less( world->cache + middle, key ) == KF_TRUE )
        {
            low = middle + 1u;
        }
        else
        {
            high = middle;
        }
    }
    if( low == world->cache_count || kf_internal_contact_less( key, world->cache + low ) == KF_TRUE )
    {
        return NULL;
    }
    return world->cache + low;
}

/* Contact A is the right shape's body, B the left one. Impulses carried from
   the previous step are spread evenly over the new points. */
static void kf_dynamics_prepare_manifold( kf_world_t * world, kf_solver_manifold_t * solver, const kf_manifold_t * manifold, size_t left_shape, size_t right_shape )
{
    size_t body_a = world->shape_objects[right_shape];
    size_t body_b = world->shape_objects[left_shape];
    const kf_object_slot_t * object_a = world->objects + body_a;
    const kf_object_slot_t * object_b = world->objects + body_b;
    const kf_solver_body_t * a = world->bodies + body_a;
    const kf_solver_body_t * b = world->bodies + body_b;
    const kf_contact_snapshot_t * cached;
    kf_matrix_t frame = kf_internal_matrix( a->start_rotation );
    uint32_t index;
    memset( solver, 0, sizeof(*solver) );
    solver->body_a = body_a;
    solver->body_b = body_b;
    solver->carried.left_object = world->shapes[left_shape].object_id;
    solver->carried.left_shape = world->shapes[left_shape].desc.id;
    solver->carried.right_object = world->shapes[right_shape].object_id;
    solver->carried.right_shape = world->shapes[right_shape].desc.id;
    cached = kf_dynamics_cached( world, &solver->carried );
    solver->began = (kf_bool_t)(cached == NULL || cached->touching == KF_FALSE);
    solver->normal = manifold->normal;
    kf_dynamics_tangents( solver->normal, solver->tangents );
    solver->friction = kf_fixed_min( object_a->desc.friction, object_b->desc.friction );
    solver->restitution = kf_fixed_max( object_a->desc.restitution, object_b->desc.restitution );
    solver->fixed = (kf_bool_t)(object_a->desc.type == KF_BODY_STATIC || object_b->desc.type == KF_BODY_STATIC);
    solver->count = manifold->count;
    solver->carried.count = manifold->count;
    for( index = 0; index != manifold->count; ++index )
    {
        const kf_manifold_point_t * source = manifold->points + index;
        kf_solver_point_t * point = solver->points + index;
        kf_vec3_t offset;
        point->anchor_a = kf_vec3_sub( source->point_right, a->center );
        point->anchor_b = kf_vec3_sub( source->point_left, b->center );
        offset = kf_vec3_sub( point->anchor_b, point->anchor_a );
        point->base_separation = (int64_t)source->separation - ((int64_t)offset.x * solver->normal.x + (int64_t)offset.y * solver->normal.y + (int64_t)offset.z * solver->normal.z) / KF_DYNAMICS_ONE;
        point->normal_mass = kf_dynamics_effective_mass( a, b, point->anchor_a, point->anchor_b, solver->normal );
        point->tangent_mass[0] = kf_dynamics_effective_mass( a, b, point->anchor_a, point->anchor_b, solver->tangents[0] );
        point->tangent_mass[1] = kf_dynamics_effective_mass( a, b, point->anchor_a, point->anchor_b, solver->tangents[1] );
        point->relative_velocity = kf_dynamics_relative_velocity( a, b, point->anchor_a, point->anchor_b, solver->normal );
        solver->carried.points[index].anchor = kf_internal_matrix_apply_transpose( &frame, point->anchor_a );
        if( cached != NULL )
        {
            const kf_contact_point_snapshot_t * match = NULL;
            int64_t best = (int64_t)KF_DYNAMICS_MATCH_DISTANCE * KF_DYNAMICS_MATCH_DISTANCE;
            uint32_t candidate;
            uint32_t tangent;
            for( candidate = 0; candidate != cached->count; ++candidate )
            {
                int64_t x = (int64_t)cached->points[candidate].anchor.x - solver->carried.points[index].anchor.x;
                int64_t y = (int64_t)cached->points[candidate].anchor.y - solver->carried.points[index].anchor.y;
                int64_t z = (int64_t)cached->points[candidate].anchor.z - solver->carried.points[index].anchor.z;
                int64_t square;
                if( x <= -KF_DYNAMICS_MATCH_DISTANCE || x >= KF_DYNAMICS_MATCH_DISTANCE )
                {
                    continue;
                }
                if( y <= -KF_DYNAMICS_MATCH_DISTANCE || y >= KF_DYNAMICS_MATCH_DISTANCE )
                {
                    continue;
                }
                if( z <= -KF_DYNAMICS_MATCH_DISTANCE || z >= KF_DYNAMICS_MATCH_DISTANCE )
                {
                    continue;
                }
                square = x * x + y * y + z * z;
                if( square < best )
                {
                    best = square;
                    match = cached->points + candidate;
                }
            }
            if( match == NULL )
            {
                continue;
            }
            point->normal_impulse = match->normal_impulse;
            for( tangent = 0; tangent != 2u; ++tangent )
            {
                kf_vec3_t direction = solver->tangents[tangent];
                point->tangent_impulse[tangent] = kf_internal_add_saturated( kf_internal_add_saturated( kf_internal_mul_shift( match->tangent_impulse[0], direction.x, 16 ), kf_internal_mul_shift( match->tangent_impulse[1], direction.y, 16 ) ), kf_internal_mul_shift( match->tangent_impulse[2], direction.z, 16 ) );
            }
        }
    }
}

static void kf_dynamics_sift_manifolds( kf_solver_manifold_t * manifolds, size_t root, size_t count )
{
    while( root < count / 2u )
    {
        size_t child = root * 2u + 1u;
        kf_solver_manifold_t swap;
        if( child + 1u < count && kf_internal_contact_less( &manifolds[child].carried, &manifolds[child + 1u].carried ) == KF_TRUE )
        {
            ++child;
        }
        if( kf_internal_contact_less( &manifolds[root].carried, &manifolds[child].carried ) == KF_FALSE )
        {
            return;
        }
        swap = manifolds[root];
        manifolds[root] = manifolds[child];
        manifolds[child] = swap;
        root = child;
    }
}

/* Scans report pairs in scan order; the solver visits them in canonical shape
   pair order, so results do not depend on which body found a pair. */
static void kf_dynamics_sort_manifolds( kf_world_t * world )
{
    size_t root = world->manifold_count / 2u;
    size_t end = world->manifold_count;
    while( root != 0 )
    {
        kf_dynamics_sift_manifolds( world->manifolds, --root, world->manifold_count );
    }
    while( end > 1u )
    {
        kf_solver_manifold_t swap = world->manifolds[0];
        world->manifolds[0] = world->manifolds[--end];
        world->manifolds[end] = swap;
        kf_dynamics_sift_manifolds( world->manifolds, 0, end );
    }
}

static kf_status_t kf_dynamics_collide( kf_world_t * world, const kf_dynamics_context_t * context )
{
    size_t index;
    kf_status_t status = kf_internal_refresh_bounds( world );
    if( status != KF_OK )
    {
        return status;
    }
    for( index = 0; index != world->shape_count; ++index )
    {
        const kf_solver_body_t * body = world->bodies + world->shape_objects[index];
        kf_aabb_t bounds = kf_dynamics_expand( world->shape_bounds[index], context->speculative );
        if( (body->flags & (KF_SOLVER_DYNAMIC | KF_SOLVER_MOVING)) != 0 )
        {
            int64_t displacement[3];
            int64_t rotation;
            kf_dynamics_predict( world, body, displacement, &rotation );
            bounds = kf_dynamics_expand( bounds, rotation );
            if( displacement[0] < 0 ) bounds.minimum.x = kf_dynamics_narrow( bounds.minimum.x + displacement[0] );
            else bounds.maximum.x = kf_dynamics_narrow( bounds.maximum.x + displacement[0] );
            if( displacement[1] < 0 ) bounds.minimum.y = kf_dynamics_narrow( bounds.minimum.y + displacement[1] );
            else bounds.maximum.y = kf_dynamics_narrow( bounds.maximum.y + displacement[1] );
            if( displacement[2] < 0 ) bounds.minimum.z = kf_dynamics_narrow( bounds.minimum.z + displacement[2] );
            else bounds.maximum.z = kf_dynamics_narrow( bounds.maximum.z + displacement[2] );
        }
        world->scratch_bounds[index] = bounds;
    }
    kf_dynamics_build_tree( world );
    /* Islands join dynamic bodies within contact range; moving kinematic bodies
       and awake members wake a whole island. */
    for( index = 0; index != world->shape_count; ++index )
    {
        size_t left_object = world->shape_objects[index];
        size_t count;
        size_t candidate;
        if( kf_dynamics_scanner( world, index ) == KF_FALSE )
        {
            continue;
        }
        count = kf_dynamics_candidates( world, index );
        for( candidate = 0; candidate != count; ++candidate )
        {
            size_t right_object = world->shape_objects[world->candidates[candidate]];
            const kf_object_slot_t * left = world->objects + left_object;
            const kf_object_slot_t * right = world->objects + right_object;
            if( kf_dynamics_pair_enabled( left, right ) == KF_FALSE )
            {
                continue;
            }
            if( left->desc.type == KF_BODY_DYNAMIC && right->desc.type == KF_BODY_DYNAMIC )
            {
                kf_dynamics_union( world->bodies, left_object, right_object );
            }
            else if( left->desc.type == KF_BODY_DYNAMIC && (world->bodies[right_object].flags & KF_SOLVER_MOVING) != 0 )
            {
                world->bodies[left_object].flags |= KF_SOLVER_WAKE;
                if( kf_dynamics_motion_bound( world, world->bodies + right_object ) > KF_SIZE_LIMIT - context->speculative )
                {
                    world->bodies[left_object].flags |= KF_SOLVER_INCOMING;
                }
            }
            else if( right->desc.type == KF_BODY_DYNAMIC && (world->bodies[left_object].flags & KF_SOLVER_MOVING) != 0 )
            {
                world->bodies[right_object].flags |= KF_SOLVER_WAKE;
                if( kf_dynamics_motion_bound( world, world->bodies + left_object ) > KF_SIZE_LIMIT - context->speculative )
                {
                    world->bodies[right_object].flags |= KF_SOLVER_INCOMING;
                }
            }
        }
    }
    for( index = 0; index != world->constraint_count; ++index )
    {
        const kf_constraint_slot_t * slot = world->constraints + index;
        kf_object_slot_t * left;
        kf_object_slot_t * right;
        size_t left_object;
        size_t right_object;
        if( slot->active == KF_FALSE )
        {
            continue;
        }
        left = kf_internal_object( world, slot->state.desc.left );
        right = kf_internal_object( world, slot->state.desc.right );
        if( left == NULL || right == NULL )
        {
            return KF_STALE_HANDLE;
        }
        left_object = (size_t)(left - world->objects);
        right_object = (size_t)(right - world->objects);
        if( left->desc.type == KF_BODY_DYNAMIC && right->desc.type == KF_BODY_DYNAMIC )
        {
            kf_dynamics_union( world->bodies, left_object, right_object );
        }
        else if( left->desc.type == KF_BODY_DYNAMIC && (world->bodies[right_object].flags & KF_SOLVER_MOVING) != 0 )
        {
            world->bodies[left_object].flags |= KF_SOLVER_WAKE;
        }
        else if( right->desc.type == KF_BODY_DYNAMIC && (world->bodies[left_object].flags & KF_SOLVER_MOVING) != 0 )
        {
            world->bodies[right_object].flags |= KF_SOLVER_WAKE;
        }
    }
    for( index = 0; index != world->object_count; ++index )
    {
        const kf_object_slot_t * object = world->objects + index;
        if( object->active == KF_FALSE || object->desc.type != KF_BODY_DYNAMIC )
        {
            continue;
        }
        if( object->state.sleeping == KF_FALSE || (world->bodies[index].flags & KF_SOLVER_WAKE) != 0 )
        {
            world->bodies[kf_dynamics_find( world->bodies, index )].flags |= KF_SOLVER_AWAKE_ISLAND;
        }
    }
    for( index = 0; index != world->object_count; ++index )
    {
        kf_object_slot_t * object = world->objects + index;
        kf_solver_body_t * body = world->bodies + index;
        if( object->active == KF_FALSE || object->desc.type != KF_BODY_DYNAMIC )
        {
            continue;
        }
        body->island = kf_dynamics_find( world->bodies, index );
        if( (world->bodies[body->island].flags & KF_SOLVER_AWAKE_ISLAND) == 0 )
        {
            continue;
        }
        /* Objects change only when the step commits. */
        if( object->state.sleeping == KF_TRUE )
        {
            body->flags |= KF_SOLVER_WOKEN;
        }
        body->flags |= KF_SOLVER_DYNAMIC;
    }
    world->manifold_count = 0;
    for( index = 0; index != world->shape_count; ++index )
    {
        size_t count;
        size_t candidate;
        if( kf_dynamics_scanner( world, index ) == KF_FALSE )
        {
            continue;
        }
        count = kf_dynamics_candidates( world, index );
        for( candidate = 0; candidate != count; ++candidate )
        {
            /* The lower canonical shape index is the left shape. */
            size_t other_index = world->candidates[candidate];
            size_t left_index = index < other_index ? index : other_index;
            size_t right_index = index < other_index ? other_index : index;
            size_t left_object = world->shape_objects[left_index];
            size_t right_object = world->shape_objects[right_index];
            const kf_object_slot_t * left = world->objects + left_object;
            const kf_object_slot_t * right = world->objects + right_object;
            const kf_solver_body_t * left_body = world->bodies + left_object;
            const kf_solver_body_t * right_body = world->bodies + right_object;
            kf_manifold_t manifold;
            int64_t margin = context->speculative;
            kf_status_t status;
            if( kf_dynamics_pair_enabled( left, right ) == KF_FALSE )
            {
                continue;
            }
            if( ((left_body->flags | right_body->flags) & KF_SOLVER_DYNAMIC) == 0 )
            {
                continue;
            }
            if( left->desc.type == KF_BODY_KINEMATIC )
            {
                margin = kf_dynamics_pair_margin( world, context, left_object );
            }
            else if( right->desc.type == KF_BODY_KINEMATIC )
            {
                margin = kf_dynamics_pair_margin( world, context, right_object );
            }
            status = kf_internal_shape_manifold( &world->shapes[left_index].desc.shape, left->state.pose, &world->shapes[right_index].desc.shape, right->state.pose, (kf_fixed_t)margin, &manifold );
            if( status == KF_NO_HIT )
            {
                continue;
            }
            if( status != KF_OK )
            {
                return status;
            }
            if( world->manifold_count == world->capacity.contacts )
            {
                return KF_CAPACITY_EXCEEDED;
            }
            kf_dynamics_prepare_manifold( world, world->manifolds + world->manifold_count, &manifold, left_index, right_index );
            ++world->manifold_count;
        }
    }
    kf_dynamics_sort_manifolds( world );
    return KF_OK;
}

static void kf_dynamics_prepare_joints( kf_world_t * world )
{
    size_t index;
    for( index = 0; index != world->constraint_count; ++index )
    {
        const kf_constraint_slot_t * slot = world->constraints + index;
        kf_solver_joint_t * joint = world->joints + index;
        kf_object_slot_t * left;
        kf_object_slot_t * right;
        kf_solver_body_t * a;
        kf_solver_body_t * b;
        kf_matrix_t matrix;
        int64_t difference[3];
        int64_t length;
        joint->active = KF_FALSE;
        if( slot->active == KF_FALSE )
        {
            continue;
        }
        left = kf_internal_object( world, slot->state.desc.left );
        right = kf_internal_object( world, slot->state.desc.right );
        if( left == NULL || right == NULL )
        {
            continue;
        }
        joint->body_a = (size_t)(right - world->objects);
        joint->body_b = (size_t)(left - world->objects);
        a = world->bodies + joint->body_a;
        b = world->bodies + joint->body_b;
        if( ((a->flags | b->flags) & KF_SOLVER_DYNAMIC) == 0 )
        {
            continue;
        }
        joint->active = KF_TRUE;
        joint->limit = (kf_bool_t)(slot->state.desc.type == KF_CONSTRAINT_MAX_DISTANCE);
        matrix = kf_internal_matrix( a->rotation );
        joint->anchor_a = kf_internal_matrix_apply( &matrix, kf_vec3_sub( slot->state.desc.anchor_right, a->mass.center ) );
        matrix = kf_internal_matrix( b->rotation );
        joint->anchor_b = kf_internal_matrix_apply( &matrix, kf_vec3_sub( slot->state.desc.anchor_left, b->mass.center ) );
        difference[0] = ((int64_t)b->center.x + joint->anchor_b.x) - ((int64_t)a->center.x + joint->anchor_a.x);
        difference[1] = ((int64_t)b->center.y + joint->anchor_b.y) - ((int64_t)a->center.y + joint->anchor_a.y);
        difference[2] = ((int64_t)b->center.z + joint->anchor_b.z) - ((int64_t)a->center.z + joint->anchor_a.z);
        length = kf_dynamics_length( difference );
        if( length == 0 )
        {
            joint->axis = (kf_vec3_t){KF_FIXED_SCALE, 0, 0};
        }
        else
        {
            joint->axis = (kf_vec3_t){kf_dynamics_narrow( kf_internal_div_shift( difference[0], length, 16 ) ), kf_dynamics_narrow( kf_internal_div_shift( difference[1], length, 16 ) ), kf_dynamics_narrow( kf_internal_div_shift( difference[2], length, 16 ) )};
        }
        joint->error = length - slot->state.desc.length;
        joint->mass = kf_dynamics_effective_mass( a, b, joint->anchor_a, joint->anchor_b, joint->axis );
    }
}

static void kf_dynamics_joint_impulse( kf_world_t * world, kf_solver_joint_t * joint, int64_t impulse )
{
    kf_vec3_t direction = joint->limit == KF_TRUE ? kf_vec3_neg( joint->axis ) : joint->axis;
    kf_dynamics_apply( world->bodies + joint->body_a, joint->anchor_a, direction, -impulse );
    kf_dynamics_apply( world->bodies + joint->body_b, joint->anchor_b, direction, impulse );
}

static void kf_dynamics_solve_joints( kf_world_t * world, const kf_dynamics_context_t * context, kf_bool_t use_bias )
{
    size_t index;
    for( index = 0; index != world->constraint_count; ++index )
    {
        kf_solver_joint_t * joint = world->joints + index;
        kf_solver_body_t * a;
        kf_solver_body_t * b;
        int64_t velocity;
        int64_t bias = 0;
        int64_t mass_scale = KF_DYNAMICS_ONE;
        int64_t impulse_scale = 0;
        int64_t impulse;
        if( joint->active == KF_FALSE )
        {
            continue;
        }
        a = world->bodies + joint->body_a;
        b = world->bodies + joint->body_b;
        velocity = kf_dynamics_relative_velocity( a, b, joint->anchor_a, joint->anchor_b, joint->axis );
        if( joint->limit == KF_TRUE )
        {
            /* A rope is a contact whose separation is the remaining slack. */
            int64_t slack = -joint->error;
            int64_t accumulated;
            velocity = -velocity;
            if( slack > 0 )
            {
                bias = kf_internal_mul_shift( slack, context->inverse_step, 16 );
            }
            else if( joint->elastic == KF_TRUE )
            {
                bias = kf_internal_mul_shift( joint->spring.bias_rate, slack, 16 );
                mass_scale = joint->spring.mass_scale;
                impulse_scale = joint->spring.impulse_scale;
            }
            else if( use_bias == KF_TRUE && slack < -context->slop )
            {
                bias = kf_internal_mul_shift( context->joint.bias_rate, slack + context->slop, 16 );
                mass_scale = context->joint.mass_scale;
                impulse_scale = context->joint.impulse_scale;
            }
            impulse = kf_internal_sub_saturated( -kf_internal_mul3_shift( joint->mass, velocity + bias, mass_scale, 48 ), kf_internal_mul_shift( impulse_scale, joint->impulse, 16 ) );
            accumulated = kf_internal_add_saturated( joint->impulse, impulse );
            if( accumulated < 0 )
            {
                accumulated = 0;
            }
            impulse = accumulated - joint->impulse;
            joint->impulse = accumulated;
        }
        else
        {
            if( joint->elastic == KF_TRUE )
            {
                /* A spring acts in every pass: its softness is the behavior, not a correction. */
                bias = kf_internal_mul_shift( joint->spring.bias_rate, joint->error, 16 );
                mass_scale = joint->spring.mass_scale;
                impulse_scale = joint->spring.impulse_scale;
            }
            else if( use_bias == KF_TRUE && (joint->error > context->slop || joint->error < -context->slop) )
            {
                int64_t error = joint->error > 0 ? joint->error - context->slop : joint->error + context->slop;
                bias = kf_internal_mul_shift( context->joint.bias_rate, error, 16 );
                mass_scale = context->joint.mass_scale;
                impulse_scale = context->joint.impulse_scale;
            }
            impulse = kf_internal_sub_saturated( -kf_internal_mul3_shift( joint->mass, velocity + bias, mass_scale, 48 ), kf_internal_mul_shift( impulse_scale, joint->impulse, 16 ) );
            {
                /* Apply only what the saturated accumulator actually took. */
                int64_t accumulated = kf_internal_add_saturated( joint->impulse, impulse );
                impulse = accumulated - joint->impulse;
                joint->impulse = accumulated;
            }
        }
        kf_dynamics_joint_impulse( world, joint, impulse );
    }
}

static void kf_dynamics_contact_impulse( kf_world_t * world, const kf_solver_manifold_t * manifold, const kf_solver_point_t * point, kf_vec3_t direction, int64_t impulse )
{
    kf_dynamics_apply( world->bodies + manifold->body_a, point->anchor_a, direction, -impulse );
    kf_dynamics_apply( world->bodies + manifold->body_b, point->anchor_b, direction, impulse );
}

static void kf_dynamics_warm_start( kf_world_t * world )
{
    size_t index;
    for( index = 0; index != world->manifold_count; ++index )
    {
        const kf_solver_manifold_t * manifold = world->manifolds + index;
        uint32_t point;
        for( point = 0; point != manifold->count; ++point )
        {
            const kf_solver_point_t * solver = manifold->points + point;
            kf_dynamics_contact_impulse( world, manifold, solver, manifold->normal, solver->normal_impulse );
            kf_dynamics_contact_impulse( world, manifold, solver, manifold->tangents[0], solver->tangent_impulse[0] );
            kf_dynamics_contact_impulse( world, manifold, solver, manifold->tangents[1], solver->tangent_impulse[1] );
        }
    }
    for( index = 0; index != world->constraint_count; ++index )
    {
        kf_solver_joint_t * joint = world->joints + index;
        if( joint->active == KF_TRUE )
        {
            kf_dynamics_joint_impulse( world, joint, joint->impulse );
        }
    }
}

/* Separation of a contact point after both bodies moved within the step. */
static int64_t kf_dynamics_separation( const kf_world_t * world, const kf_solver_manifold_t * manifold, const kf_solver_point_t * point )
{
    const kf_solver_body_t * a = world->bodies + manifold->body_a;
    const kf_solver_body_t * b = world->bodies + manifold->body_b;
    kf_vec3_t rotated_a = kf_internal_matrix_apply( &a->delta, point->anchor_a );
    kf_vec3_t rotated_b = kf_internal_matrix_apply( &b->delta, point->anchor_b );
    int64_t x = ((int64_t)b->delta_position.x - a->delta_position.x) + ((int64_t)rotated_b.x - rotated_a.x);
    int64_t y = ((int64_t)b->delta_position.y - a->delta_position.y) + ((int64_t)rotated_b.y - rotated_a.y);
    int64_t z = ((int64_t)b->delta_position.z - a->delta_position.z) + ((int64_t)rotated_b.z - rotated_a.z);
    return (x * manifold->normal.x + y * manifold->normal.y + z * manifold->normal.z) / KF_DYNAMICS_ONE + point->base_separation;
}

/* Coulomb friction on both tangents: the tangential impulse stays within the
   friction coefficient times the point's accumulated normal impulse. */
static void kf_dynamics_solve_friction( kf_world_t * world, const kf_solver_manifold_t * manifold, kf_solver_point_t * point )
{
    const kf_solver_body_t * a = world->bodies + manifold->body_a;
    const kf_solver_body_t * b = world->bodies + manifold->body_b;
    int64_t limit = kf_internal_mul_shift( manifold->friction, point->normal_impulse, 16 );
    int64_t next[3] = {0, 0, 0};
    int64_t length;
    uint32_t tangent;
    if( limit == 0 && point->tangent_impulse[0] == 0 && point->tangent_impulse[1] == 0 )
    {
        return;
    }
    for( tangent = 0; tangent != 2u; ++tangent )
    {
        int64_t velocity = kf_dynamics_relative_velocity( a, b, point->anchor_a, point->anchor_b, manifold->tangents[tangent] );
        next[tangent] = kf_internal_sub_saturated( point->tangent_impulse[tangent], kf_internal_mul_shift( point->tangent_mass[tangent], velocity, 32 ) );
    }
    length = kf_dynamics_length( next );
    if( length > limit )
    {
        next[0] = kf_internal_mul_div( next[0], limit, length, 0 );
        next[1] = kf_internal_mul_div( next[1], limit, length, 0 );
    }
    for( tangent = 0; tangent != 2u; ++tangent )
    {
        int64_t impulse = kf_internal_sub_saturated( next[tangent], point->tangent_impulse[tangent] );
        point->tangent_impulse[tangent] = next[tangent];
        kf_dynamics_contact_impulse( world, manifold, point, manifold->tangents[tangent], impulse );
    }
}

static void kf_dynamics_solve_manifold( kf_world_t * world, const kf_dynamics_context_t * context, kf_solver_manifold_t * manifold, kf_bool_t use_bias )
{
    kf_solver_body_t * a = world->bodies + manifold->body_a;
    kf_solver_body_t * b = world->bodies + manifold->body_b;
    const kf_dynamics_soft_t * soft = manifold->fixed == KF_TRUE ? &context->fixed : &context->contact;
    uint32_t point_index;
    for( point_index = 0; point_index != manifold->count; ++point_index )
    {
        kf_solver_point_t * point = manifold->points + point_index;
        int64_t separation = kf_dynamics_separation( world, manifold, point );
        int64_t bias = 0;
        int64_t mass_scale = KF_DYNAMICS_ONE;
        int64_t impulse_scale = 0;
        int64_t velocity;
        int64_t impulse;
        int64_t accumulated;
        if( separation > 0 )
        {
            /* Speculative: allow closing exactly the remaining gap. */
            bias = kf_internal_mul_shift( separation, context->inverse_step, 16 );
        }
        else if( use_bias == KF_TRUE )
        {
            /* Soft penetration recovery; relax passes remove its velocity. */
            bias = kf_internal_mul_shift( soft->bias_rate, separation, 16 );
            if( bias < -context->push_speed )
            {
                bias = -context->push_speed;
            }
            mass_scale = soft->mass_scale;
            impulse_scale = soft->impulse_scale;
        }
        velocity = kf_dynamics_relative_velocity( a, b, point->anchor_a, point->anchor_b, manifold->normal );
        impulse = kf_internal_sub_saturated( -kf_internal_mul3_shift( point->normal_mass, velocity + bias, mass_scale, 48 ), kf_internal_mul_shift( impulse_scale, point->normal_impulse, 16 ) );
        accumulated = kf_internal_add_saturated( point->normal_impulse, impulse );
        if( accumulated < 0 )
        {
            accumulated = 0;
        }
        impulse = accumulated - point->normal_impulse;
        point->normal_impulse = accumulated;
        if( accumulated > point->maximum_impulse )
        {
            point->maximum_impulse = accumulated;
        }
        kf_dynamics_contact_impulse( world, manifold, point, manifold->normal, impulse );
    }
    for( point_index = 0; point_index != manifold->count; ++point_index )
    {
        kf_dynamics_solve_friction( world, manifold, manifold->points + point_index );
    }
}

/* Static contacts are solved last in every pass, so pushes from other bodies,
   including kinematic ones, cannot override static geometry. */
static void kf_dynamics_solve_contacts( kf_world_t * world, const kf_dynamics_context_t * context, kf_bool_t use_bias )
{
    uint32_t pass;
    for( pass = 0; pass != 2u; ++pass )
    {
        size_t index;
        for( index = 0; index != world->manifold_count; ++index )
        {
            kf_solver_manifold_t * manifold = world->manifolds + index;
            if( manifold->fixed != (kf_bool_t)pass )
            {
                continue;
            }
            kf_dynamics_solve_manifold( world, context, manifold, use_bias );
        }
    }
}

static void kf_dynamics_restitution( kf_world_t * world )
{
    int64_t threshold = world->config.restitution_threshold;
    size_t index;
    for( index = 0; index != world->manifold_count; ++index )
    {
        kf_solver_manifold_t * manifold = world->manifolds + index;
        uint32_t point_index;
        if( manifold->restitution == 0 )
        {
            continue;
        }
        for( point_index = 0; point_index != manifold->count; ++point_index )
        {
            kf_solver_point_t * point = manifold->points + point_index;
            int64_t velocity;
            int64_t impulse;
            int64_t accumulated;
            if( point->relative_velocity > -threshold || point->maximum_impulse == 0 )
            {
                continue;
            }
            velocity = kf_dynamics_relative_velocity( world->bodies + manifold->body_a, world->bodies + manifold->body_b, point->anchor_a, point->anchor_b, manifold->normal );
            /* mass * (velocity + restitution * approach), each term from one wide product. */
            impulse = -kf_internal_add_saturated( kf_internal_mul_shift( point->normal_mass, velocity, 32 ), kf_internal_mul3_shift( point->normal_mass, manifold->restitution, point->relative_velocity, 48 ) );
            accumulated = kf_internal_add_saturated( point->normal_impulse, impulse );
            if( accumulated < 0 )
            {
                accumulated = 0;
            }
            impulse = accumulated - point->normal_impulse;
            point->normal_impulse = accumulated;
            kf_dynamics_contact_impulse( world, manifold, point, manifold->normal, impulse );
        }
    }
}

static void kf_dynamics_integrate_velocities( kf_world_t * world, const kf_dynamics_context_t * context )
{
    size_t index;
    for( index = 0; index != world->object_count; ++index )
    {
        const kf_object_slot_t * object = world->objects + index;
        kf_solver_body_t * body = world->bodies + index;
        int64_t linear;
        int64_t angular;
        uint32_t axis;
        if( (body->flags & KF_SOLVER_DYNAMIC) == 0 )
        {
            continue;
        }
        /* Implicit damping: v / (1 + damping * h). */
        linear = KF_DYNAMICS_ONE + (int64_t)object->desc.linear_damping * context->numerator / context->denominator;
        angular = KF_DYNAMICS_ONE + (int64_t)object->desc.angular_damping * context->numerator / context->denominator;
        for( axis = 0; axis != 3u; ++axis )
        {
            body->velocity[axis] += kf_dynamics_rational( body->gravity[axis], context->numerator, context->denominator, body->velocity_remainder + axis );
            body->velocity[axis] = kf_internal_div_shift( body->velocity[axis], linear, 16 );
            body->angular[axis] = kf_internal_div_shift( body->angular[axis], angular, 16 );
        }
        kf_dynamics_clamp_body( body );
    }
}

static void kf_dynamics_integrate_positions( kf_world_t * world, const kf_dynamics_context_t * context )
{
    size_t index;
    for( index = 0; index != world->object_count; ++index )
    {
        kf_solver_body_t * body = world->bodies + index;
        kf_solver_path_t * path = world->paths + index;
        int32_t * center[3] = {&body->center.x, &body->center.y, &body->center.z};
        uint32_t axis;
        if( (body->flags & (KF_SOLVER_DYNAMIC | KF_SOLVER_MOVING)) == 0 )
        {
            continue;
        }
        path->segments[body->rotation_count].center = body->center;
        path->segments[body->rotation_count].time = ((UINT64_C(1) << 32) * body->rotation_count) / world->config.substeps;
        memcpy( path->states[body->rotation_count].velocity, body->velocity, sizeof(body->velocity) );
        memcpy( path->states[body->rotation_count].angular, body->angular, sizeof(body->angular) );
        memcpy( path->states[body->rotation_count].position_remainder, body->position_remainder, sizeof(body->position_remainder) );
        memcpy( path->states[body->rotation_count].velocity_remainder, body->velocity_remainder, sizeof(body->velocity_remainder) );
        for( axis = 0; axis != 3u; ++axis )
        {
            *center[axis] = kf_dynamics_narrow( *center[axis] + kf_dynamics_rational( body->velocity[axis], context->numerator, context->denominator, body->position_remainder + axis ) );
        }
        {
            kf_rotation_segment_t * segment = path->segments + body->rotation_count++;
            kf_quat_t integrated;
            segment->rotation = body->rotation;
            for( axis = 0; axis != 3u; ++axis )
            {
                segment->vector[axis] = kf_internal_mul_div( body->angular[axis], context->numerator, context->denominator, 14 );
            }
            body->rotation_distance += kf_dynamics_length( segment->vector );
            if( body->angular[0] != 0 || body->angular[1] != 0 || body->angular[2] != 0 )
            {
                body->rotation = kf_internal_quat_rotate( body->rotation, segment->vector );
                body->delta = kf_internal_matrix( kf_internal_quat_multiply( body->rotation, kf_internal_quat_conjugate( body->start_rotation ) ) );
            }
            integrated = body->rotation;
            kf_dynamics_contain( world, index, segment->rotation );
            if( body->rotation.x != integrated.x || body->rotation.y != integrated.y || body->rotation.z != integrated.z || body->rotation.w != integrated.w )
            {
                body->rotation_distance -= kf_dynamics_length( segment->vector );
                segment->vector[0] = 0;
                segment->vector[1] = 0;
                segment->vector[2] = 0;
            }
        }
        body->delta_position = kf_vec3_sub( body->center, body->start_center );
    }
}

static kf_status_t kf_dynamics_solve( kf_world_t * world, const kf_dynamics_context_t * context )
{
    uint32_t substep;
    size_t index;
    for( index = 0; index != world->constraint_count; ++index )
    {
        const kf_constraint_desc_t * desc = &world->constraints[index].state.desc;
        kf_solver_joint_t * joint = world->joints + index;
        joint->impulse = world->constraints[index].state.impulse;
        joint->elastic = (kf_bool_t)(desc->hertz > 0);
        joint->spring = kf_dynamics_softness( desc->hertz, desc->damping_ratio, context->step );
    }
    for( substep = 0; substep != world->config.substeps; ++substep )
    {
        uint32_t iteration;
        kf_dynamics_integrate_velocities( world, context );
        kf_dynamics_prepare_joints( world );
        kf_dynamics_warm_start( world );
        for( iteration = 0; iteration != world->config.solver_iterations; ++iteration )
        {
            kf_dynamics_solve_joints( world, context, KF_TRUE );
            kf_dynamics_solve_contacts( world, context, KF_TRUE );
        }
        kf_dynamics_integrate_positions( world, context );
        for( iteration = 0; iteration != KF_DYNAMICS_RELAX_ITERATIONS; ++iteration )
        {
            kf_dynamics_solve_joints( world, context, KF_FALSE );
            kf_dynamics_solve_contacts( world, context, KF_FALSE );
        }
    }
    /* Carry the converged impulses, before restitution adds rebound. */
    for( index = 0; index != world->manifold_count; ++index )
    {
        kf_solver_manifold_t * manifold = world->manifolds + index;
        uint32_t point;
        for( point = 0; point != manifold->count; ++point )
        {
            const kf_solver_point_t * solver = manifold->points + point;
            kf_contact_point_snapshot_t * carried = manifold->carried.points + point;
            uint32_t tangent;
            if( solver->normal_impulse > 0 || kf_dynamics_separation( world, manifold, solver ) <= context->slop )
            {
                manifold->carried.touching = KF_TRUE;
            }
            carried->normal_impulse = solver->normal_impulse;
            for( tangent = 0; tangent != 2u; ++tangent )
            {
                kf_vec3_t direction = manifold->tangents[tangent];
                carried->tangent_impulse[0] = kf_internal_add_saturated( carried->tangent_impulse[0], kf_internal_mul_shift( solver->tangent_impulse[tangent], direction.x, 16 ) );
                carried->tangent_impulse[1] = kf_internal_add_saturated( carried->tangent_impulse[1], kf_internal_mul_shift( solver->tangent_impulse[tangent], direction.y, 16 ) );
                carried->tangent_impulse[2] = kf_internal_add_saturated( carried->tangent_impulse[2], kf_internal_mul_shift( solver->tangent_impulse[tangent], direction.z, 16 ) );
            }
        }
        manifold->began = (kf_bool_t)(manifold->began == KF_TRUE && manifold->carried.touching == KF_TRUE);
    }
    kf_dynamics_restitution( world );
    for( index = 0; index != world->object_count; ++index )
    {
        kf_dynamics_clamp_body( world->bodies + index );
    }
    return KF_OK;
}

/* Motion of a body from an absolute tick fraction to the end of the tick. */
static kf_motion_t kf_dynamics_motion( const kf_world_t * world, const kf_solver_body_t * body, uint64_t time )
{
    kf_motion_t motion;
    kf_motion_t path;
    uint64_t span = (UINT64_C(1) << 32) - body->path_start;
    uint64_t fraction = span == 0 || time <= body->path_start ? 0 : (uint64_t)kf_internal_div_shift( (int64_t)(time - body->path_start), (int64_t)span, 32 );
    memset( &path, 0, sizeof(path) );
    memset( &motion, 0, sizeof(motion) );
    path.rotation_start = body->path_rotation;
    path.rotation_end = body->end_rotation;
    if( body->rotation_distance != 0 )
    {
        path.rotation_segments = kf_dynamics_path( world, body )->segments;
        path.rotation_count = body->rotation_count;
    }
    path.center_start = body->path_center;
    path.center_end = body->end_center;
    path.translation_segments = kf_dynamics_path( world, body )->segments;
    path.translation_count = body->rotation_count;
    path.rotation_fraction_start = body->path_start;
    motion.local_center = body->mass.center;
    motion.extent = body->mass.extent;
    motion.center_start = kf_internal_motion_center( &path, fraction );
    motion.center_end = body->end_center;
    motion.translation_segments = path.translation_segments;
    motion.translation_count = path.translation_count;
    motion.rotation_start = kf_internal_motion_rotation( &path, fraction );
    motion.rotation_end = body->end_rotation;
    motion.rotation_segments = path.rotation_segments;
    motion.rotation_count = path.rotation_count;
    motion.rotation_fraction_start = time;
    return motion;
}

static kf_aabb_t kf_dynamics_motion_bounds( const kf_motion_t * motion )
{
    kf_aabb_t bounds;
    uint32_t index;
    bounds.minimum = (kf_vec3_t){kf_fixed_min( motion->center_start.x, motion->center_end.x ), kf_fixed_min( motion->center_start.y, motion->center_end.y ), kf_fixed_min( motion->center_start.z, motion->center_end.z )};
    bounds.maximum = (kf_vec3_t){kf_fixed_max( motion->center_start.x, motion->center_end.x ), kf_fixed_max( motion->center_start.y, motion->center_end.y ), kf_fixed_max( motion->center_start.z, motion->center_end.z )};
    for( index = 0; index != motion->translation_count; ++index )
    {
        kf_vec3_t center = motion->translation_segments[index].center;
        if( motion->translation_segments[index].time < motion->rotation_fraction_start )
        {
            continue;
        }
        bounds.minimum = (kf_vec3_t){kf_fixed_min( bounds.minimum.x, center.x ), kf_fixed_min( bounds.minimum.y, center.y ), kf_fixed_min( bounds.minimum.z, center.z )};
        bounds.maximum = (kf_vec3_t){kf_fixed_max( bounds.maximum.x, center.x ), kf_fixed_max( bounds.maximum.y, center.y ), kf_fixed_max( bounds.maximum.z, center.z )};
    }
    return kf_dynamics_expand( bounds, (int64_t)motion->extent + KF_GEOMETRY_TOLERANCE + (int64_t)kf_internal_magnitude( motion->center_curve.x ) + (int64_t)kf_internal_magnitude( motion->center_curve.y ) + (int64_t)kf_internal_magnitude( motion->center_curve.z ) );
}

/* A discrete contact covers exactly one leaf-shape pair. Other leaves of a
   compound body still need their own continuous collision test. */
static kf_bool_t kf_dynamics_has_manifold( const kf_world_t * world, size_t own_shape, size_t other_shape )
{
    const kf_shape_slot_t * own = world->shapes + own_shape;
    const kf_shape_slot_t * other = world->shapes + other_shape;
    kf_contact_snapshot_t key;
    size_t low = 0;
    size_t high = world->manifold_count;
    kf_bool_t ordered = (kf_bool_t)(own_shape < other_shape);
    memset( &key, 0, sizeof(key) );
    key.left_object = ordered == KF_TRUE ? own->object_id : other->object_id;
    key.left_shape = ordered == KF_TRUE ? own->desc.id : other->desc.id;
    key.right_object = ordered == KF_TRUE ? other->object_id : own->object_id;
    key.right_shape = ordered == KF_TRUE ? other->desc.id : own->desc.id;
    while( low != high )
    {
        size_t middle = low + (high - low) / 2u;
        if( kf_internal_contact_less( &world->manifolds[middle].carried, &key ) == KF_TRUE )
        {
            low = middle + 1u;
        }
        else
        {
            high = middle;
        }
    }
    if( low == world->manifold_count )
    {
        return KF_FALSE;
    }
    return (kf_bool_t)(kf_internal_contact_less( &key, &world->manifolds[low].carried ) == KF_FALSE);
}

/* Earliest impact between a fast body's shapes and one other shape. */
static kf_status_t kf_dynamics_event_pair( kf_world_t * world, const kf_dynamics_context_t * context, size_t body_index, const kf_motion_t * motion, size_t other_shape, uint64_t time )
{
    kf_solver_body_t * body = world->bodies + body_index;
    size_t other_index = world->shape_objects[other_shape];
    kf_motion_t other_motion;
    int64_t margin = context->speculative;
    size_t begin;
    size_t end;
    size_t own;
    if( other_index == body_index || kf_dynamics_pair_enabled( world->objects + body_index, world->objects + other_index ) == KF_FALSE )
    {
        return KF_OK;
    }
    if( world->objects[other_index].desc.type == KF_BODY_KINEMATIC )
    {
        margin = kf_dynamics_pair_margin( world, context, other_index );
    }
    other_motion = kf_dynamics_motion( world, world->bodies + other_index, time );
    kf_internal_shape_range( world, world->objects[body_index].desc.id, &begin, &end );
    for( own = begin; own != end; ++own )
    {
        uint64_t fraction;
        uint64_t event_time;
        kf_fixed_t initial;
        kf_status_t status;
        if( body->path_start == 0 && kf_dynamics_has_manifold( world, own, other_shape ) == KF_TRUE )
        {
            continue;
        }
        status = kf_internal_shape_toi( &world->shapes[own].desc.shape, motion, &world->shapes[other_shape].desc.shape, &other_motion, (kf_fixed_t)context->slop, &fraction, &initial );
        if( status == KF_NO_HIT )
        {
            continue;
        }
        if( status != KF_OK )
        {
            return status;
        }
        /* Pairs already within the solver's speculative range belong to the solver. */
        if( initial <= margin )
        {
            continue;
        }
        event_time = time + (uint64_t)kf_internal_mul_shift( (int64_t)((UINT64_C(1) << 32) - time), (int64_t)fraction, 32 );
        if( event_time > body->event_time )
        {
            continue;
        }
        if( event_time == body->event_time && (other_shape > body->event_other_shape || (other_shape == body->event_other_shape && own >= body->event_shape)) )
        {
            continue;
        }
        body->event_time = event_time;
        body->event_shape = own;
        body->event_other_shape = other_shape;
        body->event_other = other_index;
    }
    return KF_OK;
}

/* Earliest impact of a fast body from the given tick fraction onward. Slow and
   static shapes come from the swept leaves; fast shapes follow current paths. */
static kf_status_t kf_dynamics_find_event( kf_world_t * world, const kf_dynamics_context_t * context, size_t body_index, uint64_t time )
{
    kf_solver_body_t * body = world->bodies + body_index;
    kf_motion_t motion;
    kf_aabb_t bounds;
    size_t stack[64];
    size_t depth = 0;
    size_t index;
    body->event_time = KF_DYNAMICS_NO_EVENT;
    if( (body->flags & KF_SOLVER_FAST) == 0 || world->node_count == 0 )
    {
        return KF_OK;
    }
    motion = kf_dynamics_motion( world, body, time );
    bounds = kf_dynamics_motion_bounds( &motion );
    stack[depth++] = 0;
    while( depth != 0 )
    {
        const kf_index_node_t * node = world->nodes + stack[--depth];
        kf_status_t status;
        if( node->object_id == world->objects[body_index].desc.id )
        {
            continue;
        }
        if( kf_internal_bounds_overlap( node->bounds, bounds ) == KF_FALSE )
        {
            continue;
        }
        if( node->shape_index == SIZE_MAX )
        {
            stack[depth++] = node->right;
            stack[depth++] = node->left;
            continue;
        }
        if( (world->bodies[world->shape_objects[node->shape_index]].flags & KF_SOLVER_CONTINUOUS) != 0 )
        {
            continue;
        }
        status = kf_dynamics_event_pair( world, context, body_index, &motion, node->shape_index, time );
        if( status != KF_OK )
        {
            return status;
        }
    }
    for( index = 0; index != world->shape_count; ++index )
    {
        size_t other_index = world->shape_objects[index];
        const kf_solver_body_t * other = world->bodies + other_index;
        kf_status_t status;
        if( (other->flags & KF_SOLVER_CONTINUOUS) == 0 || other == body )
        {
            continue;
        }
        if( kf_dynamics_pair_enabled( world->objects + body_index, world->objects + other_index ) == KF_FALSE )
        {
            continue;
        }
        if( kf_internal_bounds_overlap( other->sweep, bounds ) == KF_FALSE )
        {
            continue;
        }
        status = kf_dynamics_event_pair( world, context, body_index, &motion, index, time );
        if( status != KF_OK )
        {
            return status;
        }
    }
    return KF_OK;
}

/* An impact sees the velocity that integrated its actual substep, including
   the forces, damping, clamps and contact impulses already applied there. */
static void kf_dynamics_sample_path( const kf_world_t * world, kf_solver_body_t * body, uint64_t time, kf_vec3_t center )
{
    const kf_solver_path_t * path = kf_dynamics_path( world, body );
    uint32_t index = 0;
    const kf_rotation_segment_t * segment;
    const kf_dynamics_segment_state_t * state;
    int64_t fraction;
    int64_t denominator = (int64_t)world->config.step.denominator * world->config.substeps;
    int64_t difference[3];
    uint32_t axis;
    if( body->rotation_count == 0 )
    {
        return;
    }
    while( index + 1u < body->rotation_count && path->segments[index + 1u].time <= time )
    {
        ++index;
    }
    segment = path->segments + index;
    state = path->states + index;
    fraction = (int64_t)(time - segment->time) * world->config.substeps;
    difference[0] = (int64_t)center.x - segment->center.x;
    difference[1] = (int64_t)center.y - segment->center.y;
    difference[2] = (int64_t)center.z - segment->center.z;
    for( axis = 0; axis != 3u; ++axis )
    {
        int64_t elapsed = kf_internal_mul_shift( state->velocity[axis] * world->config.step.numerator, fraction, 32 );
        body->velocity[axis] = state->velocity[axis];
        body->angular[axis] = state->angular[axis];
        body->velocity_remainder[axis] = state->velocity_remainder[axis];
        body->position_remainder[axis] = (state->position_remainder[axis] + elapsed - difference[axis] * denominator) % denominator;
    }
}

static kf_status_t kf_dynamics_respond( kf_world_t * world, const kf_dynamics_context_t * context, size_t body_index, size_t shape_index, size_t other_index, size_t other_shape, uint64_t time, kf_pose_t pose, kf_vec3_t center, kf_pose_t other_pose, kf_vec3_t other_center, kf_bool_t impact );

static void kf_dynamics_sift_indices( size_t * values, size_t root, size_t count );

static kf_status_t kf_dynamics_support_pair( kf_world_t * world, const kf_dynamics_context_t * context, size_t body_index, size_t shape_index, size_t other_shape, uint64_t time, kf_pose_t pose )
{
    size_t other_index = world->shape_objects[other_shape];
    kf_motion_t other_motion = kf_dynamics_motion( world, world->bodies + other_index, time );
    kf_pose_t other_pose = kf_internal_motion_pose( &other_motion, 0 );
    kf_status_t status = kf_dynamics_respond( world, context, body_index, shape_index, other_index, other_shape, time, pose, world->bodies[body_index].center, other_pose, other_motion.center_start, KF_FALSE );
    return status;
}

/* Replaying one impact must retain every current static/kinematic support,
   including contacts handled by the original solver and earlier CCD impacts.
   The swept tree and candidate scratch keep this bounded without a contact list. */
static kf_status_t kf_dynamics_support( kf_world_t * world, const kf_dynamics_context_t * context, kf_solver_body_t * body, uint64_t time )
{
    size_t body_index = (size_t)(body - world->bodies);
    size_t stack[64];
    size_t depth = 0;
    size_t count = 0;
    size_t begin;
    size_t end;
    size_t lower = 0;
    size_t root;
    size_t remaining;
    uint32_t iteration;
    kf_aabb_t bounds = {body->center, body->center};
    kf_motion_t motion;
    kf_pose_t pose;
    bounds = kf_dynamics_expand( bounds, (int64_t)body->mass.extent + context->speculative + context->slop + KF_GEOMETRY_TOLERANCE );
    if( world->node_count != 0 ) stack[depth++] = 0;
    while( depth != 0 )
    {
        const kf_index_node_t * node = world->nodes + stack[--depth];
        size_t other_index;
        if( node->object_id == world->objects[body_index].desc.id )
        {
            continue;
        }
        if( kf_internal_bounds_overlap( bounds, node->bounds ) == KF_FALSE )
        {
            continue;
        }
        if( node->shape_index == SIZE_MAX )
        {
            stack[depth++] = node->right;
            stack[depth++] = node->left;
            continue;
        }
        other_index = world->shape_objects[node->shape_index];
        if( world->objects[other_index].desc.type == KF_BODY_DYNAMIC )
        {
            continue;
        }
        if( kf_dynamics_pair_enabled( world->objects + body_index, world->objects + other_index ) == KF_FALSE )
        {
            continue;
        }
        world->candidates[count++] = node->shape_index;
    }
    /* Solve in canonical shape order, independently of BVH traversal. */
    for( root = count / 2u; root != 0; )
    {
        kf_dynamics_sift_indices( world->candidates, --root, count );
    }
    for( remaining = count; remaining > 1u; )
    {
        size_t swap = world->candidates[0];
        world->candidates[0] = world->candidates[--remaining];
        world->candidates[remaining] = swap;
        kf_dynamics_sift_indices( world->candidates, 0, remaining );
    }
    memset( &motion, 0, sizeof(motion) );
    motion.center_start = body->center;
    motion.rotation_start = body->rotation;
    motion.local_center = body->mass.center;
    pose = kf_internal_motion_pose( &motion, 0 );
    kf_internal_shape_range( world, world->objects[body_index].desc.id, &begin, &end );
    while( lower != count && world->candidates[lower] < begin ) ++lower;
    for( iteration = 0; iteration != KF_DYNAMICS_RESPONSE_ITERATIONS; ++iteration )
    {
        size_t candidate;
        size_t shape_index;
        /* Bodies' shapes are contiguous: lower partners are the left shape of
           each pair, and this body's shapes precede every higher partner. */
        for( candidate = 0; candidate != lower; ++candidate )
        {
            size_t other_shape = world->candidates[candidate];
            for( shape_index = begin; shape_index != end; ++shape_index )
            {
                kf_status_t status = kf_dynamics_support_pair( world, context, body_index, shape_index, other_shape, time, pose );
                if( status != KF_OK )
                {
                    return status;
                }
            }
        }
        for( shape_index = begin; shape_index != end; ++shape_index )
        {
            for( candidate = lower; candidate != count; ++candidate )
            {
                size_t other_shape = world->candidates[candidate];
                kf_status_t status = kf_dynamics_support_pair( world, context, body_index, shape_index, other_shape, time, pose );
                if( status != KF_OK )
                {
                    return status;
                }
            }
        }
    }
    return KF_OK;
}

/* Continue the current substep with the impact velocity, then replay the normal
   force/damping/clamp integration at each remaining solver boundary. */
static kf_status_t kf_dynamics_restart_path( kf_world_t * world, const kf_dynamics_context_t * context, kf_solver_body_t * body, uint64_t time, kf_vec3_t center, kf_quat_t rotation, kf_bool_t stop )
{
    kf_motion_t motion;
    kf_solver_path_t * path = kf_dynamics_path( world, body );
    kf_vec3_t saved_center = body->center;
    kf_quat_t saved_rotation = body->rotation;
    int64_t numerator = world->config.step.numerator;
    int64_t denominator = (int64_t)world->config.step.denominator * world->config.substeps;
    const kf_body_desc_t * desc = &world->objects[body - world->bodies].desc;
    uint64_t current = time;
    uint32_t boundary = 1;
    kf_bool_t first = KF_TRUE;
    body->path_start = time;
    body->path_center = center;
    body->path_rotation = rotation;
    body->end_center = center;
    body->end_rotation = rotation;
    body->rotation_count = 0;
    body->rotation_distance = 0;
    if( stop == KF_TRUE )
    {
        body->flags &= ~KF_SOLVER_FAST;
        motion = kf_dynamics_motion( world, body, time );
        body->sweep = kf_dynamics_motion_bounds( &motion );
        return KF_OK;
    }
    body->center = center;
    body->rotation = rotation;
    while( current < (UINT64_C(1) << 32) )
    {
        kf_rotation_segment_t * segment;
        kf_dynamics_segment_state_t * state;
        int32_t * component[3] = {&body->center.x, &body->center.y, &body->center.z};
        uint64_t end;
        uint64_t start;
        int64_t fraction;
        kf_quat_t integrated;
        uint32_t axis;
        while( ((UINT64_C(1) << 32) * boundary) / world->config.substeps <= current )
        {
            ++boundary;
        }
        end = ((UINT64_C(1) << 32) * boundary) / world->config.substeps;
        start = ((UINT64_C(1) << 32) * (boundary - 1u)) / world->config.substeps;
        fraction = current == start ? INT64_C(4294967296) : (int64_t)(end - current) * world->config.substeps;
        if( first == KF_FALSE )
        {
            int64_t linear = KF_DYNAMICS_ONE + (int64_t)desc->linear_damping * numerator / denominator;
            int64_t angular = KF_DYNAMICS_ONE + (int64_t)desc->angular_damping * numerator / denominator;
            for( axis = 0; axis != 3u; ++axis )
            {
                body->velocity[axis] += kf_dynamics_rational( body->gravity[axis], numerator, denominator, body->velocity_remainder + axis );
                body->velocity[axis] = kf_internal_div_shift( body->velocity[axis], linear, 16 );
                body->angular[axis] = kf_internal_div_shift( body->angular[axis], angular, 16 );
            }
            kf_dynamics_clamp_body( body );
        }
        {
            kf_status_t status = kf_dynamics_support( world, context, body, current );
            if( status != KF_OK )
            {
                body->center = saved_center;
                body->rotation = saved_rotation;
                return status;
            }
        }
        segment = path->segments + body->rotation_count;
        state = path->states + body->rotation_count++;
        segment->center = body->center;
        segment->time = current;
        segment->rotation = body->rotation;
        memcpy( state->velocity, body->velocity, sizeof(body->velocity) );
        memcpy( state->angular, body->angular, sizeof(body->angular) );
        memcpy( state->position_remainder, body->position_remainder, sizeof(body->position_remainder) );
        memcpy( state->velocity_remainder, body->velocity_remainder, sizeof(body->velocity_remainder) );
        for( axis = 0; axis != 3u; ++axis )
        {
            int64_t total = kf_internal_mul_shift( body->velocity[axis] * numerator, fraction, 32 ) + body->position_remainder[axis];
            *component[axis] = kf_dynamics_narrow( *component[axis] + total / denominator );
            body->position_remainder[axis] = total % denominator;
            /* (angular * numerator * fraction >> 32 << 14) / denominator, in one division. */
            segment->vector[axis] = kf_internal_mul_div( body->angular[axis] * numerator, fraction, denominator * (INT64_C(1) << 18), 0 );
        }
        body->rotation_distance += kf_dynamics_length( segment->vector );
        body->rotation = kf_internal_quat_rotate( body->rotation, segment->vector );
        integrated = body->rotation;
        kf_dynamics_contain( world, (size_t)(body - world->bodies), segment->rotation );
        if( body->rotation.x != integrated.x || body->rotation.y != integrated.y || body->rotation.z != integrated.z || body->rotation.w != integrated.w )
        {
            body->rotation_distance -= kf_dynamics_length( segment->vector );
            memset( segment->vector, 0, sizeof(segment->vector) );
        }
        current = end;
        first = KF_FALSE;
    }
    body->end_center = body->center;
    body->end_rotation = body->rotation;
    body->center = saved_center;
    body->rotation = saved_rotation;
    motion = kf_dynamics_motion( world, body, time );
    body->sweep = kf_dynamics_motion_bounds( &motion );
    return KF_OK;
}

/* Sequential impulses on the impact manifold; obstacles without the dynamic
   flag act with infinite mass and their prescribed velocity. */
static kf_status_t kf_dynamics_respond( kf_world_t * world, const kf_dynamics_context_t * context, size_t body_index, size_t shape_index, size_t other_index, size_t other_shape, uint64_t time, kf_pose_t pose, kf_vec3_t center, kf_pose_t other_pose, kf_vec3_t other_center, kf_bool_t impact )
{
    kf_solver_body_t * a = world->bodies + other_index;
    kf_solver_body_t * b = world->bodies + body_index;
    const kf_object_slot_t * object_a = world->objects + other_index;
    const kf_object_slot_t * object_b = world->objects + body_index;
    kf_manifold_t manifold;
    kf_solver_manifold_t solver;
    int64_t targets[KF_MANIFOLD_MAX_POINTS];
    kf_vec3_t saved_a = a->center;
    kf_vec3_t saved_b = b->center;
    int64_t saved_velocity[3];
    int64_t saved_angular[3];
    uint32_t iteration;
    uint32_t index;
    kf_status_t status = kf_internal_shape_unbounded_manifold( &world->shapes[shape_index].desc.shape, pose, &world->shapes[other_shape].desc.shape, other_pose, (kf_fixed_t)(context->speculative + context->slop), &manifold );
    if( status == KF_NO_HIT )
    {
        return KF_OK;
    }
    if( status != KF_OK )
    {
        return status;
    }
    a->center = other_center;
    b->center = center;
    if( object_a->desc.type == KF_BODY_KINEMATIC )
    {
        const kf_solver_path_t * path = kf_dynamics_path( world, a );
        uint32_t segment = 0;
        memcpy( saved_velocity, a->velocity, sizeof(saved_velocity) );
        memcpy( saved_angular, a->angular, sizeof(saved_angular) );
        if( a->rotation_count != 0 )
        {
            while( segment + 1u < a->rotation_count && path->segments[segment + 1u].time <= time )
            {
                ++segment;
            }
            memcpy( a->velocity, path->states[segment].velocity, sizeof(a->velocity) );
            memcpy( a->angular, path->states[segment].angular, sizeof(a->angular) );
        }
    }
    memset( &solver, 0, sizeof(solver) );
    solver.body_a = other_index;
    solver.body_b = body_index;
    solver.normal = manifold.normal;
    kf_dynamics_tangents( solver.normal, solver.tangents );
    solver.friction = kf_fixed_min( object_a->desc.friction, object_b->desc.friction );
    solver.restitution = kf_fixed_max( object_a->desc.restitution, object_b->desc.restitution );
    solver.count = manifold.count;
    for( index = 0; index != manifold.count; ++index )
    {
        kf_solver_point_t * point = solver.points + index;
        point->anchor_a = kf_vec3_sub( manifold.points[index].point_right, a->center );
        point->anchor_b = kf_vec3_sub( manifold.points[index].point_left, b->center );
        point->normal_mass = kf_dynamics_effective_mass( a, b, point->anchor_a, point->anchor_b, solver.normal );
        point->tangent_mass[0] = kf_dynamics_effective_mass( a, b, point->anchor_a, point->anchor_b, solver.tangents[0] );
        point->tangent_mass[1] = kf_dynamics_effective_mass( a, b, point->anchor_a, point->anchor_b, solver.tangents[1] );
        point->relative_velocity = kf_dynamics_relative_velocity( a, b, point->anchor_a, point->anchor_b, solver.normal );
        targets[index] = impact == KF_TRUE && point->relative_velocity < -world->config.restitution_threshold ? -kf_internal_mul_shift( solver.restitution, point->relative_velocity, 16 ) : 0;
    }
    for( iteration = 0; iteration != (impact == KF_TRUE ? KF_DYNAMICS_RESPONSE_ITERATIONS : 1u); ++iteration )
    {
        for( index = 0; index != solver.count; ++index )
        {
            kf_solver_point_t * point = solver.points + index;
            int64_t velocity = kf_dynamics_relative_velocity( a, b, point->anchor_a, point->anchor_b, solver.normal );
            int64_t impulse = -kf_internal_mul_shift( point->normal_mass, velocity - targets[index], 32 );
            int64_t accumulated = kf_internal_add_saturated( point->normal_impulse, impulse );
            if( accumulated < 0 )
            {
                accumulated = 0;
            }
            impulse = accumulated - point->normal_impulse;
            point->normal_impulse = accumulated;
            kf_dynamics_contact_impulse( world, &solver, point, solver.normal, impulse );
        }
        for( index = 0; index != solver.count; ++index )
        {
            kf_dynamics_solve_friction( world, &solver, solver.points + index );
        }
    }
    a->center = saved_a;
    b->center = saved_b;
    kf_dynamics_clamp_body( a );
    kf_dynamics_clamp_body( b );
    if( object_a->desc.type == KF_BODY_KINEMATIC )
    {
        memcpy( a->velocity, saved_velocity, sizeof(a->velocity) );
        memcpy( a->angular, saved_angular, sizeof(a->angular) );
    }
    return KF_OK;
}

static kf_status_t kf_dynamics_continuous( kf_world_t * world, const kf_dynamics_context_t * context )
{
    kf_bool_t any = KF_FALSE;
    size_t index;
    for( index = 0; index != world->object_count; ++index )
    {
        kf_solver_body_t * body = world->bodies + index;
        int64_t difference[3] = {(int64_t)body->center.x - body->start_center.x, (int64_t)body->center.y - body->start_center.y, (int64_t)body->center.z - body->start_center.z};
        int64_t motion;
        int64_t threshold;
        int64_t rotation_motion;
        uint32_t segment;
        body->path_start = 0;
        body->path_center = body->start_center;
        body->path_rotation = body->start_rotation;
        body->end_center = body->center;
        body->end_rotation = body->rotation;
        if( (body->flags & KF_SOLVER_DYNAMIC) == 0 )
        {
            continue;
        }
        /* A returning path can have a zero endpoint chord. Bound each axis by
           its largest recorded excursion before testing the travel threshold. */
        for( segment = 0; segment != body->rotation_count; ++segment )
        {
            kf_vec3_t center = world->paths[index].segments[segment].center;
            int64_t excursion[3] = {(int64_t)center.x - body->start_center.x, (int64_t)center.y - body->start_center.y, (int64_t)center.z - body->start_center.z};
            uint32_t axis;
            for( axis = 0; axis != 3u; ++axis )
            {
                if( kf_internal_magnitude( excursion[axis] ) > kf_internal_magnitude( difference[axis] ) )
                {
                    difference[axis] = excursion[axis];
                }
            }
        }
        motion = kf_dynamics_length( difference );
        rotation_motion = kf_internal_mul_shift( body->rotation_distance, body->mass.extent, 30 );
        if( body->rotation_distance >= KF_INTERNAL_PI )
        {
            motion += rotation_motion + 1;
        }
        else
        {
            int64_t chord = kf_dynamics_rotation_bound( body->start_rotation, body->rotation, body->mass.extent );
            motion += chord > rotation_motion ? chord : rotation_motion;
        }
        threshold = body->mass.thickness / 2;
        if( threshold < context->speculative )
        {
            threshold = context->speculative;
        }
        if( motion > threshold || (body->flags & KF_SOLVER_INCOMING) != 0 )
        {
            body->flags |= KF_SOLVER_FAST | KF_SOLVER_CONTINUOUS;
            any = KF_TRUE;
        }
    }
    if( any == KF_FALSE )
    {
        return KF_OK;
    }
    for( index = 0; index != world->object_count; ++index )
    {
        kf_solver_body_t * body = world->bodies + index;
        if( (body->flags & KF_SOLVER_FAST) != 0 )
        {
            kf_motion_t motion = kf_dynamics_motion( world, body, 0 );
            body->sweep = kf_dynamics_motion_bounds( &motion );
        }
    }
    /* Leaves hold swept bounds of every slow shape over the whole tick. */
    for( index = 0; index != world->shape_count; ++index )
    {
        const kf_solver_body_t * body = world->bodies + world->shape_objects[index];
        kf_aabb_t bounds = world->shape_bounds[index];
        if( (body->flags & (KF_SOLVER_DYNAMIC | KF_SOLVER_MOVING)) != 0 )
        {
            kf_motion_t motion = kf_dynamics_motion( world, body, 0 );
            kf_aabb_t swept = kf_dynamics_motion_bounds( &motion );
            bounds.minimum = (kf_vec3_t){kf_fixed_min( bounds.minimum.x, swept.minimum.x ), kf_fixed_min( bounds.minimum.y, swept.minimum.y ), kf_fixed_min( bounds.minimum.z, swept.minimum.z )};
            bounds.maximum = (kf_vec3_t){kf_fixed_max( bounds.maximum.x, swept.maximum.x ), kf_fixed_max( bounds.maximum.y, swept.maximum.y ), kf_fixed_max( bounds.maximum.z, swept.maximum.z )};
        }
        world->scratch_bounds[index] = bounds;
    }
    kf_dynamics_build_tree( world );
    for( index = 0; index != world->object_count; ++index )
    {
        kf_status_t status = kf_dynamics_find_event( world, context, index, 0 );
        if( status != KF_OK )
        {
            return status;
        }
    }
    for( ;; )
    {
        size_t selected = SIZE_MAX;
        kf_solver_body_t * body;
        kf_solver_body_t * other;
        size_t other_index;
        uint64_t time;
        kf_motion_t motion;
        kf_motion_t other_motion;
        kf_pose_t pose;
        kf_pose_t other_pose;
        kf_bool_t other_fast;
        kf_bool_t stop;
        kf_status_t status;
        for( index = 0; index != world->object_count; ++index )
        {
            const kf_solver_body_t * candidate = world->bodies + index;
            if( candidate->event_time == KF_DYNAMICS_NO_EVENT )
            {
                continue;
            }
            if( selected == SIZE_MAX || candidate->event_time < world->bodies[selected].event_time )
            {
                selected = index;
            }
        }
        if( selected == SIZE_MAX )
        {
            break;
        }
        body = world->bodies + selected;
        other_index = body->event_other;
        other = world->bodies + other_index;
        time = body->event_time;
        other_fast = (kf_bool_t)((other->flags & KF_SOLVER_FAST) != 0);
        motion = kf_dynamics_motion( world, body, time );
        other_motion = kf_dynamics_motion( world, other, time );
        pose = kf_internal_motion_pose( &motion, 0 );
        other_pose = kf_internal_motion_pose( &other_motion, 0 );
        kf_dynamics_sample_path( world, body, time, motion.center_start );
        if( other_fast == KF_TRUE )
        {
            kf_dynamics_sample_path( world, other, time, other_motion.center_start );
        }
        stop = (kf_bool_t)(body->events >= world->config.ccd_iterations);
        if( other_fast == KF_TRUE && other->events >= world->config.ccd_iterations ) stop = KF_TRUE;
        if( (other->flags & KF_SOLVER_DYNAMIC) != 0 && other_fast == KF_FALSE ) stop = KF_TRUE;
        if( stop == KF_TRUE )
        {
            /* Budget exhausted, or a slow dynamic body: stop here and let the
               next step's contact solve the impact with both masses. Both fast
               paths stop if either participant exhausted its impact budget. */
            status = kf_dynamics_restart_path( world, context, body, time, motion.center_start, motion.rotation_start, KF_TRUE );
            if( status != KF_OK )
            {
                return status;
            }
            if( other_fast == KF_TRUE )
            {
                status = kf_dynamics_restart_path( world, context, other, time, other_motion.center_start, other_motion.rotation_start, KF_TRUE );
                if( status != KF_OK )
                {
                    return status;
                }
            }
        }
        else
        {
            status = kf_dynamics_respond( world, context, selected, body->event_shape, other_index, body->event_other_shape, time, pose, motion.center_start, other_pose, other_motion.center_start, KF_TRUE );
            if( status != KF_OK )
            {
                return status;
            }
            ++body->events;
            status = kf_dynamics_restart_path( world, context, body, time, motion.center_start, motion.rotation_start, KF_FALSE );
            if( status != KF_OK )
            {
                return status;
            }
            if( other_fast == KF_TRUE )
            {
                ++other->events;
                status = kf_dynamics_restart_path( world, context, other, time, other_motion.center_start, other_motion.rotation_start, KF_FALSE );
                if( status != KF_OK )
                {
                    return status;
                }
            }
        }
        for( index = 0; index != world->object_count; ++index )
        {
            kf_solver_body_t * affected = world->bodies + index;
            if( index != selected && !(other_fast == KF_TRUE && index == other_index) )
            {
                kf_bool_t existing;
                kf_bool_t overlaps;
                /* Only fast bodies have events and swept bounds. */
                if( (affected->flags & KF_SOLVER_FAST) == 0 )
                {
                    continue;
                }
                existing = (kf_bool_t)(affected->event_time != KF_DYNAMICS_NO_EVENT && (affected->event_other == selected || (other_fast == KF_TRUE && affected->event_other == other_index)));
                overlaps = kf_internal_bounds_overlap( affected->sweep, body->sweep );
                if( other_fast == KF_TRUE && kf_internal_bounds_overlap( affected->sweep, other->sweep ) == KF_TRUE )
                {
                    overlaps = KF_TRUE;
                }
                if( existing == KF_FALSE && overlaps == KF_FALSE )
                {
                    continue;
                }
            }
            /* A changed path can create an event where none existed before. */
            status = kf_dynamics_find_event( world, context, index, time );
            if( status != KF_OK )
            {
                return status;
            }
        }
    }
    for( index = 0; index != world->object_count; ++index )
    {
        kf_solver_body_t * body = world->bodies + index;
        if( (body->flags & KF_SOLVER_DYNAMIC) == 0 )
        {
            continue;
        }
        body->center = body->end_center;
        body->rotation = body->end_rotation;
        kf_dynamics_contain( world, index, body->start_rotation );
    }
    return KF_OK;
}

/* Pairs that were not evaluated, because none of their bodies was awake, keep
   their carried impulses; evaluated pairs are replaced by this step's result. */
static kf_bool_t kf_dynamics_cache_dormant( kf_world_t * world, const kf_contact_snapshot_t * entry )
{
    kf_object_slot_t * left = kf_internal_object_id( world, entry->left_object );
    kf_object_slot_t * right = kf_internal_object_id( world, entry->right_object );
    if( left == NULL )
    {
        return KF_FALSE;
    }
    if( right == NULL )
    {
        return KF_FALSE;
    }
    /* Descriptor edits can make a sleeping pair ineligible for response. */
    if( kf_dynamics_pair_enabled( left, right ) == KF_FALSE )
    {
        return KF_FALSE;
    }
    return (kf_bool_t)(((world->bodies[left - world->objects].flags | world->bodies[right - world->objects].flags) & KF_SOLVER_DYNAMIC) == 0);
}

static kf_status_t kf_dynamics_prepare_cache( kf_world_t * world, size_t * cache_count )
{
    size_t old_index = 0;
    size_t manifold_index = 0;
    size_t count = 0;
    while( old_index != world->cache_count || manifold_index != world->manifold_count )
    {
        const kf_contact_snapshot_t * entry;
        if( manifold_index == world->manifold_count || (old_index != world->cache_count && kf_internal_contact_less( world->cache + old_index, &world->manifolds[manifold_index].carried ) == KF_TRUE) )
        {
            entry = world->cache + old_index++;
            if( kf_dynamics_cache_dormant( world, entry ) == KF_FALSE )
            {
                continue;
            }
        }
        else
        {
            entry = &world->manifolds[manifold_index++].carried;
            if( old_index != world->cache_count && kf_internal_contact_less( entry, world->cache + old_index ) == KF_FALSE )
            {
                ++old_index;
            }
        }
        if( count == world->capacity.contacts )
        {
            return KF_CAPACITY_EXCEEDED;
        }
        world->next_cache[count++] = *entry;
    }
    *cache_count = count;
    return KF_OK;
}

/* Commits the step; every fallible stage ran before, so objects either take the
   whole step or keep their previous state. */
static void kf_dynamics_finish( kf_world_t * world, size_t cache_count )
{
    size_t index;
    for( index = 0; index != world->object_count; ++index )
    {
        kf_object_slot_t * object = world->objects + index;
        kf_solver_body_t * body = world->bodies + index;
        kf_matrix_t matrix;
        kf_vec3_t offset;
        uint32_t axis;
        if( (body->flags & KF_SOLVER_WOKEN) != 0 )
        {
            kf_dynamics_wake( object );
        }
        if( (body->flags & (KF_SOLVER_DYNAMIC | KF_SOLVER_MOVING)) == 0 )
        {
            continue;
        }
        if( body->rotation.x != body->start_rotation.x || body->rotation.y != body->start_rotation.y || body->rotation.z != body->start_rotation.z || body->rotation.w != body->start_rotation.w )
        {
            object->state.pose.rotation = kf_internal_rotation( body->rotation );
        }
        matrix = kf_internal_matrix( kf_internal_quat( object->state.pose.rotation ) );
        offset = kf_internal_matrix_apply( &matrix, body->mass.center );
        object->state.pose.position = (kf_vec3_t){kf_dynamics_narrow( (int64_t)body->center.x - offset.x ), kf_dynamics_narrow( (int64_t)body->center.y - offset.y ), kf_dynamics_narrow( (int64_t)body->center.z - offset.z )};
        object->state.velocity = (kf_vec3_t){(kf_fixed_t)body->velocity[0], (kf_fixed_t)body->velocity[1], (kf_fixed_t)body->velocity[2]};
        object->state.angular_velocity = (kf_vec3_t){(kf_fixed_t)body->angular[0], (kf_fixed_t)body->angular[1], (kf_fixed_t)body->angular[2]};
        for( axis = 0; axis != 3u; ++axis )
        {
            object->state.position_remainder[axis] = body->position_remainder[axis];
            object->state.velocity_remainder[axis] = body->velocity_remainder[axis];
        }
        if( object->state.has_target == KF_TRUE )
        {
            object->state.pose = object->state.target;
            object->state.has_target = KF_FALSE;
        }
        /* Contained poses keep shapes in range; a failure only leaves the cache stale. */
        (void)kf_internal_update_object_bounds( world, object );
    }
    {
        kf_contact_snapshot_t * swap = world->cache;
        world->cache = world->next_cache;
        world->next_cache = swap;
        world->cache_count = cache_count;
    }
    world->reports_valid = KF_TRUE;
    /* Islands sleep together once every member stayed slow long enough. */
    for( index = 0; index != world->object_count; ++index )
    {
        world->bodies[index].island_counter = UINT32_MAX;
    }
    for( index = 0; index != world->object_count; ++index )
    {
        kf_object_slot_t * object = world->objects + index;
        kf_solver_body_t * body = world->bodies + index;
        int64_t angular;
        if( (body->flags & KF_SOLVER_DYNAMIC) == 0 )
        {
            continue;
        }
        angular = kf_internal_mul_shift( kf_dynamics_length( body->angular ), body->mass.extent, 16 );
        if( world->config.sleep_ticks == 0 || kf_dynamics_length( body->velocity ) > world->config.sleep_speed || angular > world->config.sleep_speed )
        {
            object->state.sleep_counter = 0;
        }
        else if( object->state.sleep_counter < world->config.sleep_ticks )
        {
            ++object->state.sleep_counter;
        }
        if( object->state.sleep_counter < world->bodies[body->island].island_counter )
        {
            world->bodies[body->island].island_counter = object->state.sleep_counter;
        }
    }
    for( index = 0; index != world->object_count; ++index )
    {
        kf_object_slot_t * object = world->objects + index;
        kf_solver_body_t * body = world->bodies + index;
        if( (body->flags & KF_SOLVER_DYNAMIC) == 0 || world->config.sleep_ticks == 0 )
        {
            continue;
        }
        if( world->bodies[body->island].island_counter < world->config.sleep_ticks )
        {
            continue;
        }
        object->state.sleeping = KF_TRUE;
        object->state.velocity = (kf_vec3_t){0, 0, 0};
        object->state.angular_velocity = (kf_vec3_t){0, 0, 0};
    }
    /* Queries rebuild the index lazily from the cached bounds. */
    world->index_dirty = KF_TRUE;
}

static void kf_dynamics_sift_indices( size_t * values, size_t root, size_t count )
{
    while( root < count / 2u )
    {
        size_t child = root * 2u + 1u;
        size_t swap;
        if( child + 1u < count && values[child] < values[child + 1u] )
        {
            ++child;
        }
        if( values[root] >= values[child] )
        {
            return;
        }
        swap = values[root];
        values[root] = values[child];
        values[child] = swap;
        root = child;
    }
}

/* Index shapes overlapping a trigger shape's bounds, in canonical order. */
static size_t kf_dynamics_trigger_candidates( kf_world_t * world, size_t trigger_index )
{
    kf_aabb_t bounds = world->shape_bounds[trigger_index];
    size_t stack[64];
    size_t depth = 0;
    size_t count = 0;
    size_t root;
    size_t end;
    if( world->node_count == 0 )
    {
        return 0;
    }
    stack[depth++] = 0;
    while( depth != 0 )
    {
        const kf_index_node_t * node = world->nodes + stack[--depth];
        if( node->object_id == world->shapes[trigger_index].object_id )
        {
            continue;
        }
        if( kf_internal_bounds_overlap( bounds, node->bounds ) == KF_FALSE )
        {
            continue;
        }
        if( node->shape_index == SIZE_MAX )
        {
            stack[depth++] = node->right;
            stack[depth++] = node->left;
            continue;
        }
        if( world->shape_objects[node->shape_index] != world->shape_objects[trigger_index] )
        {
            world->candidates[count++] = node->shape_index;
        }
    }
    for( root = count / 2u; root != 0; )
    {
        kf_dynamics_sift_indices( world->candidates, --root, count );
    }
    for( end = count; end > 1u; )
    {
        size_t swap = world->candidates[0];
        world->candidates[0] = world->candidates[--end];
        world->candidates[end] = swap;
        kf_dynamics_sift_indices( world->candidates, 0, end );
    }
    return count;
}

/* Keeps the current trigger overlaps as if the step had not changed them. */
static void kf_dynamics_keep_triggers( kf_world_t * world )
{
    if( world->trigger_count != 0 )
    {
        memcpy( world->previous_triggers, world->triggers, world->trigger_count * sizeof(*world->triggers) );
    }
    world->previous_trigger_count = world->trigger_count;
}

/* Overlaps of trigger shapes at the poses after the step, in canonical order. The
   new set replaces the current one, which remains for the step's events. */
static void kf_dynamics_detect_triggers( kf_world_t * world )
{
    kf_trigger_overlap_t * found = world->previous_triggers;
    kf_bool_t any = KF_FALSE;
    size_t count = 0;
    size_t index;
    world->trigger_overflow = 0;
    for( index = 0; index != world->object_count && any == KF_FALSE; ++index )
    {
        const kf_object_slot_t * object = world->objects + index;
        any = (kf_bool_t)(object->active == KF_TRUE && object->desc.trigger == KF_TRUE && object->desc.mask != 0);
    }
    /* Contained poses keep every shape in range, so the rebuild succeeds. */
    world->triggers_valid = KF_TRUE;
    if( any == KF_TRUE && kf_world_rebuild_index( world ) != KF_OK )
    {
        kf_dynamics_keep_triggers( world );
        return;
    }
    for( index = 0; any == KF_TRUE && index != world->shape_count; ++index )
    {
        const kf_object_slot_t * trigger = world->objects + world->shape_objects[index];
        size_t candidate_count;
        size_t candidate;
        if( trigger->active == KF_FALSE || trigger->desc.trigger == KF_FALSE || trigger->desc.mask == 0 )
        {
            continue;
        }
        candidate_count = kf_dynamics_trigger_candidates( world, index );
        for( candidate = 0; candidate != candidate_count; ++candidate )
        {
            size_t visitor_index = world->candidates[candidate];
            const kf_object_slot_t * visitor = world->objects + world->shape_objects[visitor_index];
            if( visitor->active == KF_FALSE || visitor->desc.trigger == KF_TRUE )
            {
                continue;
            }
            if( trigger->desc.type == KF_BODY_STATIC && visitor->desc.type == KF_BODY_STATIC )
            {
                continue;
            }
            if( (trigger->desc.category & visitor->desc.mask) == 0 || (visitor->desc.category & trigger->desc.mask) == 0 )
            {
                continue;
            }
            if( kf_internal_shape_overlap( &world->shapes[index].desc.shape, trigger->state.pose, &world->shapes[visitor_index].desc.shape, visitor->state.pose ) != KF_OK )
            {
                continue;
            }
            if( count < world->capacity.triggers )
            {
                found[count] = (kf_trigger_overlap_t){trigger->state.handle, world->shapes[index].desc.id, visitor->state.handle, world->shapes[visitor_index].desc.id};
            }
            ++count;
        }
    }
    if( count > world->capacity.triggers )
    {
        world->trigger_overflow = count;
        kf_dynamics_keep_triggers( world );
        return;
    }
    world->previous_triggers = world->triggers;
    world->previous_trigger_count = world->trigger_count;
    world->triggers = found;
    world->trigger_count = count;
}

kf_status_t kf_world_step( kf_world_t * world )
{
    kf_dynamics_context_t context;
    kf_status_t status;
    size_t cache_count = 0;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->tick == UINT64_MAX )
    {
        return KF_OVERFLOW;
    }
    /* Manifolds are rebuilt below; reports return once the step completes. */
    world->reports_valid = KF_FALSE;
    world->triggers_valid = KF_FALSE;
    {
        size_t shape_index;
        size_t object_index = 0;
        for( shape_index = 0; shape_index != world->shape_count; ++shape_index )
        {
            uint32_t id = world->shapes[shape_index].object_id;
            while( object_index != world->object_count && world->objects[object_index].desc.id < id )
            {
                ++object_index;
            }
            if( object_index == world->object_count )
            {
                return KF_STALE_HANDLE;
            }
            if( world->objects[object_index].desc.id != id )
            {
                return KF_STALE_HANDLE;
            }
            world->shape_objects[shape_index] = object_index;
        }
    }
    context = kf_dynamics_context( world );
    status = kf_dynamics_prepare( world );
    if( status == KF_OK )
    {
        status = kf_dynamics_collide( world, &context );
    }
    if( status == KF_OK )
    {
        status = kf_dynamics_solve( world, &context );
    }
    if( status == KF_OK )
    {
        status = kf_dynamics_continuous( world, &context );
    }
    if( status == KF_OK )
    {
        status = kf_dynamics_prepare_cache( world, &cache_count );
    }
    if( status != KF_OK )
    {
        /* Only capacity and invariant failures reach here, before any object or
           constraint changed; physical limits are enforced per body instead. */
        world->index_dirty = KF_TRUE;
        return status;
    }
    {
        size_t index;
        for( index = 0; index != world->constraint_count; ++index )
        {
            if( world->joints[index].active == KF_TRUE )
            {
                world->constraints[index].state.impulse = kf_dynamics_narrow( world->joints[index].impulse );
            }
        }
    }
    kf_dynamics_finish( world, cache_count );
    kf_dynamics_detect_triggers( world );
    ++world->tick;
    return KF_OK;
}

uint64_t kf_world_tick( const kf_world_t * world )
{
    if( world == NULL )
    {
        return 0;
    }
    return world->tick;
}

kf_status_t kf_world_contact_reports( const kf_world_t * world, kf_contact_report_t * reports, size_t capacity, size_t * required )
{
    size_t count = 0;
    size_t index;
    if( world == NULL || required == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    *required = 0;
    if( capacity != 0 && reports == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->reports_valid == KF_FALSE )
    {
        return KF_NO_HIT;
    }
    for( index = 0; index != world->manifold_count; ++index )
    {
        const kf_solver_manifold_t * manifold = world->manifolds + index;
        const kf_solver_body_t * a = world->bodies + manifold->body_a;
        kf_contact_report_t * report;
        int64_t sum[3] = {0, 0, 0};
        int64_t approach = 0;
        uint32_t point;
        if( manifold->carried.touching == KF_FALSE )
        {
            continue;
        }
        if( count++ >= capacity )
        {
            continue;
        }
        report = reports + count - 1u;
        for( point = 0; point != manifold->count; ++point )
        {
            const kf_solver_point_t * solver = manifold->points + point;
            sum[0] += solver->anchor_a.x;
            sum[1] += solver->anchor_a.y;
            sum[2] += solver->anchor_a.z;
            if( -solver->relative_velocity > approach )
            {
                approach = -solver->relative_velocity;
            }
        }
        report->left = world->objects[manifold->body_b].state.handle;
        report->left_shape = manifold->carried.left_shape;
        report->right = world->objects[manifold->body_a].state.handle;
        report->right_shape = manifold->carried.right_shape;
        report->normal = manifold->normal;
        report->point = (kf_vec3_t){kf_dynamics_narrow( (int64_t)a->start_center.x + sum[0] / manifold->count ), kf_dynamics_narrow( (int64_t)a->start_center.y + sum[1] / manifold->count ), kf_dynamics_narrow( (int64_t)a->start_center.z + sum[2] / manifold->count )};
        report->approach_speed = kf_dynamics_narrow( approach );
        report->began = manifold->began;
    }
    *required = count;
    if( count > capacity )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    return count == 0 ? KF_NO_HIT : KF_OK;
}

kf_status_t kf_world_trigger_events( const kf_world_t * world, kf_trigger_event_t * events, size_t capacity, size_t * required )
{
    size_t previous = 0;
    size_t current = 0;
    size_t count = 0;
    if( world == NULL || required == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    *required = 0;
    if( capacity != 0 && events == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->triggers_valid == KF_FALSE )
    {
        return KF_NO_HIT;
    }
    if( world->trigger_overflow != 0 )
    {
        *required = world->trigger_overflow;
        return KF_CAPACITY_EXCEEDED;
    }
    while( previous != world->previous_trigger_count || current != world->trigger_count )
    {
        kf_trigger_event_t event;
        int32_t order;
        if( previous == world->previous_trigger_count )
        {
            order = 1;
        }
        else if( current == world->trigger_count )
        {
            order = -1;
        }
        else
        {
            order = kf_internal_trigger_compare( world->previous_triggers + previous, world->triggers + current );
        }
        if( order == 0 )
        {
            ++previous;
            ++current;
            continue;
        }
        event.overlap = order < 0 ? world->previous_triggers[previous++] : world->triggers[current++];
        event.began = (kf_bool_t)(order > 0);
        if( count < capacity )
        {
            events[count] = event;
        }
        ++count;
    }
    *required = count;
    if( count > capacity )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    return count == 0 ? KF_NO_HIT : KF_OK;
}
