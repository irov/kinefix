#include "kinefix/kinefix.h"
#include "../src/kinefix_internal.h"

#include <limits.h>
#include <stdio.h>

#define CHECK(Expression) do { if( (Expression) == 0 ) { fprintf( stderr, "rotation:%d: %s\n", __LINE__, #Expression ); return 1; } } while( 0 )

static int check_vector( kf_vec3_t actual, kf_vec3_t expected, int64_t tolerance )
{
    int64_t difference[3] = {(int64_t)actual.x - expected.x, (int64_t)actual.y - expected.y, (int64_t)actual.z - expected.z};
    uint32_t axis;
    for( axis = 0; axis != 3u; ++axis )
    {
        CHECK( difference[axis] >= -tolerance && difference[axis] <= tolerance );
    }
    return 0;
}

static int test_axis_scale( void )
{
    const kf_vec3_t axes[] = {{1, 1, 0}, {1, 1, 1}, {1, 2, 3}, {-3, 1, -2}, {0, 0, -1}, {7, -11, 13}};
    const int32_t scales[] = {1, 3, 16, 65536, 1234567, 100000000};
    const kf_fixed_t angles[] = {1, 102944, -102944, 205887, 411775, -1234567, INT32_MAX, INT32_MIN};
    const kf_vec3_t basis[] = {{KF_FIXED_SCALE, 0, 0}, {0, KF_FIXED_SCALE, 0}, {0, 0, KF_FIXED_SCALE}};
    size_t axis;
    size_t scale;
    size_t angle;
    size_t component;
    for( axis = 0; axis != sizeof(axes) / sizeof(axes[0]); ++axis )
    {
        for( angle = 0; angle != sizeof(angles) / sizeof(angles[0]); ++angle )
        {
            kf_rotation_t reference = kf_rotation_from_axis_angle( axes[axis], angles[angle] );
            CHECK( kf_rotation_valid( reference ) == KF_TRUE );
            for( scale = 0; scale != sizeof(scales) / sizeof(scales[0]); ++scale )
            {
                kf_vec3_t direction = {axes[axis].x * scales[scale], axes[axis].y * scales[scale], axes[axis].z * scales[scale]};
                kf_rotation_t actual = kf_rotation_from_axis_angle( direction, angles[angle] );
                CHECK( kf_rotation_valid( actual ) == KF_TRUE );
                for( component = 0; component != sizeof(basis) / sizeof(basis[0]); ++component )
                {
                    /* Each rotated unit vector is rounded to public Q16. */
                    CHECK( check_vector( kf_rotation_apply( actual, basis[component] ), kf_rotation_apply( reference, basis[component] ), 1 ) == 0 );
                }
            }
        }
    }
    return 0;
}

static int test_axis_angle( void )
{
    const kf_vec3_t vector = {0, 0, KF_FIXED_SCALE};
    const kf_rotation_t identity = {0, 0, 0};
    kf_rotation_t rotation = kf_rotation_from_axis_angle( (kf_vec3_t){1, 1, 0}, 102944 );
    CHECK( check_vector( kf_rotation_apply( rotation, vector ), (kf_vec3_t){46341, -46341, 0}, 1 ) == 0 );
    rotation = kf_rotation_from_axis_angle( (kf_vec3_t){1, 1, 1}, 102944 );
    CHECK( check_vector( kf_rotation_apply( rotation, vector ), (kf_vec3_t){59683, -15992, 21845}, 1 ) == 0 );
    rotation = kf_rotation_from_axis_angle( (kf_vec3_t){INT32_MIN, 0, 0}, 102944 );
    CHECK( check_vector( kf_rotation_apply( rotation, vector ), (kf_vec3_t){0, KF_FIXED_SCALE, 0}, 1 ) == 0 );
    rotation = kf_rotation_from_axis_angle( (kf_vec3_t){0, 0, 0}, INT32_MIN );
    CHECK( rotation.x == identity.x && rotation.y == identity.y && rotation.z == identity.z );
    rotation = kf_rotation_from_axis_angle( (kf_vec3_t){INT32_MIN, INT32_MAX, 1}, 0 );
    CHECK( rotation.x == identity.x && rotation.y == identity.y && rotation.z == identity.z );
    return 0;
}

static int test_wide_arithmetic( void )
{
    const uint64_t cases[][5] = {
        {0, 0, 0, 1, 0},
        {0, 1, 0, 1, 1},
        {0, UINT64_MAX, 0, 1, UINT64_MAX},
        {1, 0, 0, 1, UINT64_MAX},
        {1, 23, 0, 7, UINT64_C(0x2492492492492495)},
        {UINT64_MAX, UINT64_MAX, 0, UINT64_MAX, UINT64_MAX},
        {UINT64_MAX, UINT64_MAX, 1, 0, UINT64_MAX},
        {UINT64_MAX, UINT64_MAX, 2, 7, UINT64_C(0x7ffffffffffffffe)},
        {UINT64_MAX, UINT64_MAX, UINT64_C(0x8000000000000000), 0, 1},
        {UINT64_C(0x8000000000000000), 0, UINT64_C(0x8000000000000000), 1, 0},
        {UINT64_C(0x8000000000000000), 1, UINT64_C(0x8000000000000000), 0, 1},
        {UINT64_C(0x4000000000000000), UINT64_C(0x8000000000000001), 2, UINT64_C(0x8000000000000007), UINT64_C(0x1999999999999999)},
        {UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX, 1},
        {0, 0, 0, 0, UINT64_MAX},
        {UINT64_MAX, UINT64_MAX, 0, 0, UINT64_MAX},
        {UINT64_C(0x0000000100000001), 0, 0, UINT64_C(0x0000000100000001), UINT64_MAX},
        {UINT64_C(0x0000001000000000), 12345, 256, 17, UINT64_C(0x0fffffff)},
        {UINT64_C(0x7fffffffffffffff), UINT64_MAX, 0, UINT64_C(0x8000000000000001), UINT64_C(0xfffffffffffffffe)}
    };
    const kf_wide_t low = kf_internal_wide_make( 0, UINT64_MAX );
    const kf_wide_t one = kf_internal_wide_make( 0, 1 );
    const kf_wide_t high = kf_internal_wide_make( 1, 0 );
    size_t index;
    CHECK( kf_internal_wide_compare( kf_internal_wide_add( low, one ), high ) == 0 );
    CHECK( kf_internal_wide_compare( kf_internal_wide_subtract( high, one ), low ) == 0 );
    CHECK( kf_internal_wide_compare( low, high ) == -1 );
    CHECK( kf_internal_wide_compare( high, low ) == 1 );
    for( index = 0; index != sizeof(cases) / sizeof(cases[0]); ++index )
    {
        kf_wide_t numerator = kf_internal_wide_make( cases[index][0], cases[index][1] );
        kf_wide_t denominator = kf_internal_wide_make( cases[index][2], cases[index][3] );
        CHECK( kf_internal_wide_quotient( numerator, denominator ) == cases[index][4] );
    }
    return 0;
}

int main( void )
{
    CHECK( test_axis_scale() == 0 );
    CHECK( test_axis_angle() == 0 );
    CHECK( test_wide_arithmetic() == 0 );
    return 0;
}
