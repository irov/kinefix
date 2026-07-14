#ifndef KINEFIX_KINEFIX_H
#define KINEFIX_KINEFIX_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KF_VERSION_MAJOR 1
#define KF_VERSION_MINOR 2
#define KF_VERSION_PATCH 0

#define KF_FIXED_FRACTION_BITS 32
#define KF_FIXED_SCALE INT64_C(4294967296)
#define KF_FIXED_PI INT64_C(13493037704)
#define KF_FIXED_HALF_PI INT64_C(6746518852)
#define KF_FIXED_TWO_PI INT64_C(26986075409)

#define KF_ANGLE16_TURN UINT32_C(65536)
#define KF_ANGLE16_QUARTER UINT16_C(16384)
#define KF_ANGLE16_HALF UINT16_C(32768)
#define KF_ANGLE16_THREE_QUARTERS UINT16_C(49152)

typedef int64_t kf_fixed_t;
typedef uint16_t kf_angle16_t;
typedef int16_t kf_sangle16_t;
typedef uint8_t kf_fault_code_t;

#define KF_FAULT_NONE UINT8_C(0)
#define KF_FAULT_OVERFLOW UINT8_C(1)
#define KF_FAULT_DIVISION_BY_ZERO UINT8_C(2)
#define KF_FAULT_INVALID_SQUARE_ROOT UINT8_C(3)
#define KF_FAULT_INVALID_CONFIGURATION UINT8_C(4)
#define KF_FAULT_INVALID_SNAPSHOT UINT8_C(5)

typedef struct kf_fault_t
{
    kf_fault_code_t code;
    const char * operation;
} kf_fault_t;

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
    uint32_t id;
    kf_fixed_t minimum[3];
    kf_fixed_t maximum[3];
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

typedef struct kf_hit_t
{
    uint32_t object_id;
    kf_fixed_t fraction;
    kf_fixed_t distance;
    kf_vec3_t position;
    kf_vec3_t normal;
    uint8_t started_inside;
} kf_hit_t;

typedef struct kf_character_body_t
{
    kf_vec3_t position;
    kf_vec3_t velocity;
    uint8_t grounded;
} kf_character_body_t;

typedef struct kf_character_config_t
{
    kf_fixed_t radius;
    kf_fixed_t height;
    kf_fixed_t step_height;
    kf_fixed_t gravity;
    kf_fixed_t tick_seconds;
    kf_fixed_t support_epsilon;
    kf_fixed_t arena_half_extent;
} kf_character_config_t;

typedef struct kf_character_result_t
{
    kf_hit_t hit;
    kf_fixed_t previous_vertical_velocity;
    uint8_t landed;
    uint8_t hit_ceiling;
    uint8_t stepped;
    uint8_t blocked;
} kf_character_result_t;

void kf_fault_clear( kf_fault_t * fault );
void kf_fault_raise( kf_fault_t * fault, kf_fault_code_t code, const char * operation );
int kf_fault_failed( const kf_fault_t * fault );
kf_fault_t * kf_fault_bind( kf_fault_t * fault );
void kf_fault_restore( kf_fault_t * previous );

kf_fixed_t kf_fixed_from_int( int64_t value );
kf_fixed_t kf_fixed_from_decimal( const char * value );
kf_fixed_t kf_fixed_from_decimal_n( const char * value, size_t size );
double kf_fixed_to_double( kf_fixed_t value );
kf_fixed_t kf_fixed_neg( kf_fixed_t value );
kf_fixed_t kf_fixed_add( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_sub( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_mul( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_div( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_abs( kf_fixed_t value );
kf_fixed_t kf_fixed_min( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_max( kf_fixed_t left, kf_fixed_t right );
kf_fixed_t kf_fixed_clamp( kf_fixed_t value, kf_fixed_t minimum, kf_fixed_t maximum );
kf_fixed_t kf_fixed_sqrt( kf_fixed_t value );
kf_fixed_t kf_fixed_wrap_angle( kf_fixed_t angle );
void kf_fixed_sin_cos( kf_fixed_t angle, kf_fixed_t * sine, kf_fixed_t * cosine );

kf_angle16_t kf_angle16_from_fixed_radians( kf_fixed_t radians );
kf_sangle16_t kf_sangle16_from_fixed_radians( kf_fixed_t radians );
kf_fixed_t kf_angle16_to_fixed_radians( kf_angle16_t angle );
kf_fixed_t kf_sangle16_to_fixed_radians( kf_sangle16_t angle );
kf_angle16_t kf_angle16_add( kf_angle16_t angle, kf_sangle16_t delta );
kf_sangle16_t kf_sangle16_add_clamped( kf_sangle16_t angle, kf_sangle16_t delta,
    kf_sangle16_t minimum, kf_sangle16_t maximum );
void kf_angle16_sin_cos( kf_angle16_t angle, kf_fixed_t * sine, kf_fixed_t * cosine );
void kf_sangle16_sin_cos( kf_sangle16_t angle, kf_fixed_t * sine, kf_fixed_t * cosine );

kf_vec3_t kf_vec3_add( kf_vec3_t left, kf_vec3_t right );
kf_vec3_t kf_vec3_sub( kf_vec3_t left, kf_vec3_t right );
kf_vec3_t kf_vec3_mul( kf_vec3_t value, kf_fixed_t scalar );
kf_vec3_t kf_vec3_div( kf_vec3_t value, kf_fixed_t scalar );
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

int kf_aabb_overlaps_character( const kf_aabb_t * box, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height );
int kf_world_character_collides( const kf_aabb_t * boxes, size_t count, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height );
int kf_world_find_step_top( const kf_aabb_t * boxes, size_t count, kf_vec3_t candidate,
    kf_fixed_t half_width, kf_fixed_t height, kf_fixed_t current_y, kf_fixed_t maximum_step, kf_fixed_t * top );

int kf_overlap_aabb_aabb( const kf_aabb_t * left, const kf_aabb_t * right );
int kf_overlap_sphere_aabb( const kf_sphere_t * sphere, const kf_aabb_t * box );
int kf_overlap_capsule_aabb( const kf_capsule_t * capsule, const kf_aabb_t * box );
int kf_overlap_sphere_capsule( const kf_sphere_t * sphere, const kf_capsule_t * capsule );
int kf_world_overlap_capsule( const kf_aabb_t * boxes, size_t count, const kf_capsule_t * capsule, uint32_t * brush_id );

int kf_raycast_aabb( const kf_ray_t * ray, const kf_aabb_t * box, kf_hit_t * hit );
int kf_raycast_capsule( const kf_ray_t * ray, const kf_capsule_t * capsule, uint32_t object_id, kf_hit_t * hit );
int kf_world_raycast( const kf_aabb_t * boxes, size_t count, const kf_ray_t * ray, kf_hit_t * hit );
int kf_sweep_sphere_aabb_hit( const kf_sphere_t * sphere, kf_vec3_t displacement, const kf_aabb_t * box, kf_hit_t * hit );
int kf_world_sweep_sphere_hit( const kf_aabb_t * boxes, size_t count, const kf_sphere_t * sphere,
    kf_vec3_t displacement, kf_hit_t * hit );
int kf_sweep_sphere_capsule( const kf_sphere_t * sphere, kf_vec3_t displacement,
    const kf_capsule_t * capsule, uint32_t object_id, kf_hit_t * hit );
int kf_sweep_capsule_aabb( const kf_capsule_t * capsule, kf_vec3_t displacement, const kf_aabb_t * box, kf_hit_t * hit );
int kf_world_sweep_capsule( const kf_aabb_t * boxes, size_t count, const kf_capsule_t * capsule,
    kf_vec3_t displacement, kf_hit_t * hit );

int kf_sweep_sphere_aabb( kf_vec3_t start, kf_vec3_t end, kf_fixed_t radius, const kf_aabb_t * box, kf_fixed_t * hit_time );
int kf_world_sweep_sphere( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end,
    kf_fixed_t radius, kf_fixed_t * hit_time, uint32_t * brush_id );
int kf_world_line_blocked( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end,
    kf_fixed_t minimum_time, kf_fixed_t maximum_time );
void kf_character_step( kf_character_body_t * body, const kf_character_config_t * config,
    const kf_aabb_t * boxes, size_t count, kf_character_result_t * result );

#ifdef __cplusplus
}
#endif

#endif
