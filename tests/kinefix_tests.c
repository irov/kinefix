#include "kinefix/kinefix.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

static int failures = 0;

#define CHECK(Expression) \
    do \
    { \
        if( !(Expression) ) \
        { \
            fprintf( stderr, "failed line %d: %s\n", __LINE__, #Expression ); \
            ++failures; \
        } \
    } while( 0 )
#define F(Value) kf_fixed_from_double( (double)(Value) )

static void test_fixed_conversions( void )
{
    float values[] = {0.f, -0.f, 1.75f, -1.75f, FLT_MIN, -FLT_MIN, 32767.f, -32768.f};
    size_t valueIndex;

    CHECK( kf_fixed_from_double( 0.0 ) == 0 );
    CHECK( kf_fixed_from_double( -0.0 ) == 0 );
    CHECK( kf_fixed_from_double( 1.75 ) == 114688 );
    CHECK( kf_fixed_from_double( -1.75 ) == -114688 );
    CHECK( kf_fixed_from_double( 0.5 / KF_FIXED_SCALE ) == 0 );
    CHECK( kf_fixed_from_double( -0.5 / KF_FIXED_SCALE ) == 0 );
    CHECK( kf_fixed_from_double( 1.5 / KF_FIXED_SCALE ) == 1 );
    CHECK( kf_fixed_from_double( -1.5 / KF_FIXED_SCALE ) == -1 );
    CHECK( kf_fixed_from_double( DBL_MIN ) == 0 );
    CHECK( kf_fixed_from_double( -DBL_MIN ) == 0 );
    CHECK( kf_fixed_from_double( (double)INT32_MAX / KF_FIXED_SCALE ) == INT32_MAX );
    CHECK( kf_fixed_from_double( (double)INT32_MIN / KF_FIXED_SCALE ) == INT32_MIN );

    /* These doubles round to +/-1 as floats and used to lose one fixed unit. */
    CHECK( kf_fixed_from_double( 1.0 - 0x1p-25 ) == KF_FIXED_SCALE - 1 );
    CHECK( kf_fixed_from_double( -1.0 + 0x1p-25 ) == -KF_FIXED_SCALE + 1 );

    for( valueIndex = 0; valueIndex != sizeof(values) / sizeof(values[0]); ++valueIndex )
    {
        float value = values[valueIndex];
        double scaled = (double)value * KF_FIXED_SCALE;
        CHECK( kf_fixed_from_float( value ) == (kf_fixed_t)scaled );
    }

#if defined(NDEBUG)
    CHECK( kf_fixed_from_double( NAN ) == 0 );
    CHECK( kf_fixed_from_double( INFINITY ) == 0 );
    CHECK( kf_fixed_from_double( -INFINITY ) == 0 );
    CHECK( kf_fixed_from_double( DBL_MAX ) == 0 );
    CHECK( kf_fixed_from_double( -DBL_MAX ) == 0 );
    CHECK( kf_fixed_from_double( ((double)INT32_MAX + 0.25) / KF_FIXED_SCALE ) == 0 );
    CHECK( kf_fixed_from_double( ((double)INT32_MIN - 0.25) / KF_FIXED_SCALE ) == 0 );
    CHECK( kf_fixed_from_float( NAN ) == 0 );
    CHECK( kf_fixed_from_float( INFINITY ) == 0 );
    CHECK( kf_fixed_from_float( -INFINITY ) == 0 );
    CHECK( kf_fixed_from_float( FLT_MAX ) == 0 );
    CHECK( kf_fixed_from_float( -FLT_MAX ) == 0 );
#endif
}

static kf_bool_t fixed_near( kf_fixed_t left, kf_fixed_t right )
{
    return (kf_bool_t)(kf_fixed_abs( kf_fixed_sub( left, right ) ) <= F(0.001));
}

int main( void )
{
    kf_fixed_t sine;
    kf_fixed_t cosine;
    kf_pcg32_t first;
    kf_pcg32_t second;
    uint32_t index;

    test_fixed_conversions();

    CHECK( sizeof(kf_fixed_t) == 4u );
    CHECK( sizeof(kf_bool_t) == 1u );
    CHECK( KF_FALSE == 0u );
    CHECK( KF_TRUE == 1u );
    CHECK( KF_FIXED_FRACTION_BITS == 16 );
    CHECK( KF_FIXED_SCALE == INT32_C(65536) );
    CHECK( kf_fixed_from_ratio( 1, 4 ) == KF_FIXED_SCALE / 4 );
    CHECK( kf_fixed_from_ratio( -3, 2 ) == F(-1.5) );
    CHECK( kf_fixed_to_int( F(2.75) ) == 2 );
    CHECK( kf_fixed_to_int( F(-2.75) ) == -2 );
    CHECK( kf_fixed_to_float( F(2.75) ) == 2.75f );
    CHECK( kf_fixed_to_float( F(-2.75) ) == -2.75f );
    CHECK( kf_fixed_mul( F(1.5), F(-2.25) ) == F(-3.375) );
    CHECK( kf_fixed_div( F(-2.25), F(1.5) ) == F(-1.5) );
    CHECK( kf_fixed_add_mul( 1, -1, KF_FIXED_SCALE / 2 ) == 0 );
    CHECK( kf_fixed_sub_mul( -1, -1, KF_FIXED_SCALE / 2 ) == 0 );
    CHECK( kf_fixed_add_mul2( 0, 1, KF_FIXED_SCALE / 2, 1, KF_FIXED_SCALE / 2 ) == 1 );
    CHECK( kf_fixed_mul_div( F(1.5), F(2.0), F(3.0) ) == F(1.0) );
    CHECK( kf_fixed_lerp( F(-2.0), F(2.0), F(0.25) ) == F(-1.0) );
    CHECK( kf_vec3_equal( kf_vec3_add_mul( (kf_vec3_t){1, 1, 1}, (kf_vec3_t){-1, -1, -1}, KF_FIXED_SCALE / 2 ), (kf_vec3_t){0, 0, 0} ) == KF_TRUE );
    CHECK( kf_vec3_equal( kf_vec3_add_mul2( (kf_vec3_t){0, 0, 0}, (kf_vec3_t){1, 1, 1}, KF_FIXED_SCALE / 2, (kf_vec3_t){1, 1, 1}, KF_FIXED_SCALE / 2 ), (kf_vec3_t){1, 1, 1} ) == KF_TRUE );
    CHECK( kf_vec3_equal( kf_vec3_lerp( (kf_vec3_t){F(-2.0), 0, F(2.0)}, (kf_vec3_t){F(2.0), F(4.0), F(-2.0)}, F(0.25) ), (kf_vec3_t){F(-1.0), F(1.0), F(1.0)} ) == KF_TRUE );
    CHECK( kf_fixed_sqrt( kf_fixed_from_int( 9 ) ) == kf_fixed_from_int( 3 ) );
    CHECK( kf_vec3_length( (kf_vec3_t){kf_fixed_from_int( 120 ), 0, kf_fixed_from_int( 160 )} ) == kf_fixed_from_int( 200 ) );
    CHECK( fixed_near( kf_segment_point_distance_squared( (kf_vec3_t){kf_fixed_from_int( -1000 ), 0, 0}, (kf_vec3_t){kf_fixed_from_int( 1000 ), 0, 0}, (kf_vec3_t){0, 0, KF_FIXED_SCALE} ), KF_FIXED_SCALE ) );
    kf_fixed_sin_cos( KF_FIXED_PI / 4, &sine, &cosine );
    CHECK( kf_fixed_abs( kf_fixed_sub( sine, cosine ) ) < 50000 );
    CHECK( kf_angle16_from_fixed_radians( 0 ) == 0u );
    CHECK( kf_angle16_from_fixed_radians( KF_FIXED_HALF_PI ) == KF_ANGLE16_QUARTER );
    CHECK( kf_angle16_from_fixed_radians( KF_FIXED_PI ) == KF_ANGLE16_HALF );
    CHECK( kf_angle16_from_fixed_radians( KF_FIXED_TWO_PI ) == 0u );
    CHECK( kf_sangle16_from_fixed_radians( -KF_FIXED_HALF_PI ) == -INT16_C(16384) );
    CHECK( kf_angle16_to_fixed_radians( KF_ANGLE16_HALF ) == KF_FIXED_PI );
    CHECK( kf_sangle16_to_fixed_radians( -INT16_C(16384) ) == -KF_FIXED_HALF_PI );
    CHECK( kf_angle16_add( UINT16_C(65530), INT16_C(10) ) == UINT16_C(4) );
    CHECK( kf_sangle16_add_clamped( INT16_C(16000), INT16_C(1000), -INT16_C(16202), INT16_C(16202) ) == INT16_C(16202) );
    kf_angle16_sin_cos( 0u, &sine, &cosine );
    CHECK( sine == 0 && cosine == KF_FIXED_SCALE );
    kf_angle16_sin_cos( KF_ANGLE16_QUARTER, &sine, &cosine );
    CHECK( sine == KF_FIXED_SCALE && cosine == 0 );
    kf_angle16_sin_cos( KF_ANGLE16_HALF, &sine, &cosine );
    CHECK( sine == 0 && cosine == -KF_FIXED_SCALE );
    kf_angle16_sin_cos( UINT16_C(8192), &sine, &cosine );
    CHECK( sine == cosine );
    CHECK( kf_fixed_abs( kf_fixed_sub( sine, INT32_C(46340) ) ) <= INT32_C(1) );
    for( index = 0; index < KF_ANGLE16_TURN; index += 257u )
    {
        kf_fixed_t unit_length;
        kf_angle16_sin_cos( (kf_angle16_t)index, &sine, &cosine );
        unit_length = kf_fixed_add( kf_fixed_mul( sine, sine ), kf_fixed_mul( cosine, cosine ) );
        CHECK( kf_fixed_abs( kf_fixed_sub( unit_length, KF_FIXED_SCALE ) ) <= INT32_C(8) );
    }
    kf_pcg32_seed( &first, 42, 54 );
    kf_pcg32_seed( &second, 42, 54 );
    for( index = 0; index != 1000; ++index )
    {
        CHECK( kf_pcg32_next( &first ) == kf_pcg32_next( &second ) );
    }

    if( failures != 0 )
    {
        return 1;
    }
    puts( "kinefix C tests passed" );
    return 0;
}
