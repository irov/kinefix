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
    kf_collider_t floor_collider = {1u, {{-KF_FIXED_SCALE, -KF_FIXED_SCALE, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, 0, KF_FIXED_SCALE}}};
    kf_character_body_t body = {{0, 0, 0}, {0, 0, 0}, KF_TRUE};
    kf_character_config_t config = {
        KF_FIXED_SCALE / 4, KF_FIXED_SCALE, KF_FIXED_SCALE / 2,
        kf_fixed_from_int( 20 ), F(0.01), F(0.02), kf_fixed_from_int( 32 )};
    kf_character_result_t result;
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

    CHECK( kf_world_character_collides( &floor_collider, 1, body.position, config.radius, config.height ) == KF_FALSE );
    body.grounded = KF_FALSE;
    kf_character_step( &body, &config, &floor_collider, 1, &result );
    CHECK( body.grounded == KF_TRUE );
    CHECK( body.position.y == 0 );

    {
        const kf_collider_t collider = {7u, {{0, -KF_FIXED_SCALE, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, KF_FIXED_SCALE, KF_FIXED_SCALE}}};
        const kf_ray_t ray = {{-kf_fixed_from_int( 10 ), 0, 0}, {KF_FIXED_SCALE, 0, 0}, kf_fixed_from_int( 20 )};
        kf_hit_t hit;
        CHECK( kf_world_raycast( &collider, 1, &ray, &hit ) == KF_TRUE );
        CHECK( hit.object_id == 7u );
        CHECK( fixed_near( hit.fraction, F(0.5) ) );
        CHECK( fixed_near( hit.distance, kf_fixed_from_int( 10 ) ) );
        CHECK( fixed_near( hit.position.x, 0 ) );
        CHECK( hit.normal.x == -KF_FIXED_SCALE && hit.normal.y == 0 && hit.normal.z == 0 );
        CHECK( hit.started_inside == KF_FALSE );
    }

    {
        const kf_aabb_t box = {{KF_FIXED_SCALE, -KF_FIXED_SCALE, -KF_FIXED_SCALE}, {kf_fixed_from_int( 2 ), KF_FIXED_SCALE, KF_FIXED_SCALE}};
        const kf_ray_t ray = {{-kf_fixed_from_int( 1000 ), 0, 0}, {KF_FIXED_SCALE, 0, 0}, kf_fixed_from_int( 2000 )};
        kf_hit_t hit;
        CHECK( kf_raycast_aabb( &ray, &box, &hit ) == KF_TRUE );
        CHECK( fixed_near( hit.fraction, F(0.5005) ) );
        CHECK( kf_fixed_abs( kf_fixed_sub( hit.position.x, KF_FIXED_SCALE ) ) <= INT32_C(1) );
        CHECK( kf_fixed_abs( kf_fixed_sub( hit.distance, kf_fixed_from_int( 1001 ) ) ) <= INT32_C(1) );
    }

    {
        const kf_aabb_t box = {{0, -KF_FIXED_SCALE, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, KF_FIXED_SCALE, KF_FIXED_SCALE}};
        const kf_sphere_t sphere = {{-kf_fixed_from_int( 10 ), 0, 0}, KF_FIXED_SCALE};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_aabb_hit( &sphere, (kf_vec3_t){kf_fixed_from_int( 20 ), 0, 0}, &box, &hit ) == KF_TRUE );
        CHECK( fixed_near( hit.fraction, F(0.45) ) );
        CHECK( fixed_near( hit.position.x, -KF_FIXED_SCALE ) );
        CHECK( hit.normal.x == -KF_FIXED_SCALE );
    }

    {
        const kf_aabb_t corner = {{0, 0, 0}, {KF_FIXED_SCALE, kf_fixed_from_int( 3 ), KF_FIXED_SCALE}};
        kf_capsule_t capsule = {{F(1.4), 0, F(1.4)}, F(0.5), kf_fixed_from_int( 2 )};
        CHECK( kf_overlap_capsule_aabb( &capsule, &corner ) == KF_FALSE );
        capsule.position.x = F(1.3);
        capsule.position.z = F(1.3);
        CHECK( kf_overlap_capsule_aabb( &capsule, &corner ) == KF_TRUE );
    }

    {
        const kf_collider_t walls[2] = {
            {9u, {{0, 0, -KF_FIXED_SCALE}, {F(0.1), kf_fixed_from_int( 3 ), KF_FIXED_SCALE}}},
            {3u, {{0, 0, -KF_FIXED_SCALE}, {F(0.1), kf_fixed_from_int( 3 ), KF_FIXED_SCALE}}}};
        const kf_capsule_t capsule = {{-kf_fixed_from_int( 10 ), 0, 0}, F(0.5), kf_fixed_from_int( 2 )};
        kf_hit_t hit;
        CHECK( kf_world_sweep_capsule( walls, 2, &capsule, (kf_vec3_t){kf_fixed_from_int( 20 ), 0, 0}, &hit ) == KF_TRUE );
        CHECK( hit.object_id == 3u );
        CHECK( fixed_near( hit.fraction, F(0.475) ) );
        CHECK( fixed_near( hit.position.x, F(-0.5) ) );
        CHECK( hit.normal.x == -KF_FIXED_SCALE );
    }

    {
        const kf_capsule_t target = {{0, 0, 0}, F(0.5), kf_fixed_from_int( 2 )};
        const kf_sphere_t projectile = {{-kf_fixed_from_int( 1000 ), KF_FIXED_SCALE, 0}, F(0.1)};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_capsule( &projectile, (kf_vec3_t){kf_fixed_from_int( 2000 ), 0, 0}, &target, 81u, &hit ) == KF_TRUE );
        CHECK( fixed_near( hit.fraction, F(0.4997) ) );
        CHECK( fixed_near( hit.position.x, F(-0.6) ) );
    }

    {
        const kf_capsule_t target = {{0, 0, 0}, F(0.5), kf_fixed_from_int( 2 )};
        const kf_sphere_t projectile = {{-kf_fixed_from_int( 10 ), KF_FIXED_SCALE, 0}, F(0.1)};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_capsule( &projectile, (kf_vec3_t){kf_fixed_from_int( 20 ), 0, 0}, &target, 77u, &hit ) == KF_TRUE );
        CHECK( hit.object_id == 77u );
        CHECK( fixed_near( hit.fraction, F(0.47) ) );
        CHECK( fixed_near( hit.position.x, F(-0.6) ) );
        CHECK( hit.normal.x < F(-0.999) );
        CHECK( hit.started_inside == KF_FALSE );

        {
            const kf_ray_t ray = {projectile.position, {KF_FIXED_SCALE, 0, 0}, kf_fixed_from_int( 20 )};
            CHECK( kf_raycast_capsule( &ray, &target, 79u, &hit ) == KF_TRUE );
            CHECK( hit.object_id == 79u );
            CHECK( fixed_near( hit.fraction, F(0.475) ) );
            CHECK( fixed_near( hit.position.x, F(-0.5) ) );
        }
    }

    {
        const kf_capsule_t target = {{0, 0, 0}, F(0.5), kf_fixed_from_int( 2 )};
        const kf_sphere_t projectile = {{0, kf_fixed_from_int( 5 ), 0}, F(0.1)};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_capsule( &projectile, (kf_vec3_t){0, -kf_fixed_from_int( 10 ), 0}, &target, 78u, &hit ) == KF_TRUE );
        CHECK( fixed_near( hit.fraction, F(0.29) ) );
        CHECK( fixed_near( hit.position.y, F(2.1) ) );
        CHECK( hit.normal.y > F(0.999) );
    }

    {
        const kf_capsule_t target = {{0, 0, 0}, F(0.5), kf_fixed_from_int( 2 )};
        const kf_sphere_t projectile = {{kf_fixed_from_int( 100 ), kf_fixed_from_int( 5 ), 0}, F(0.1)};
        const kf_vec3_t almost_vertical = {INT32_C(10), -kf_fixed_from_int( 10 ), 0};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_capsule( &projectile, almost_vertical, &target, 80u, &hit ) == KF_FALSE );
    }

    {
        const kf_collider_t world[2] = {
            {1u, {{-kf_fixed_from_int( 100 ), -KF_FIXED_SCALE, -kf_fixed_from_int( 100 )}, {kf_fixed_from_int( 100 ), 0, kf_fixed_from_int( 100 )}}},
            {2u, {{0, 0, -KF_FIXED_SCALE}, {F(0.1), kf_fixed_from_int( 3 ), KF_FIXED_SCALE}}}};
        kf_character_body_t fast_body = {{-kf_fixed_from_int( 10 ), 0, 0}, {kf_fixed_from_int( 2000 ), 0, 0}, KF_TRUE};
        const kf_character_config_t fast_config = {
            F(0.5), kf_fixed_from_int( 2 ), F(0.4),
            kf_fixed_from_int( 20 ), F(0.01), F(0.001), kf_fixed_from_int( 100 )};
        kf_character_result_t fast_result;
        kf_character_step( &fast_body, &fast_config, world, 2, &fast_result );
        CHECK( fast_result.blocked == KF_TRUE );
        CHECK( fast_result.hit.object_id == 2u );
        CHECK( fast_body.position.x < F(-0.5) );
        CHECK( fast_body.velocity.x == 0 );
        CHECK( fast_body.grounded == KF_TRUE );
    }

    {
        const kf_collider_t world[2] = {
            {1u, {{-kf_fixed_from_int( 10 ), -KF_FIXED_SCALE, -kf_fixed_from_int( 10 )}, {kf_fixed_from_int( 10 ), 0, kf_fixed_from_int( 10 )}}},
            {2u, {{0, 0, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, F(0.25), KF_FIXED_SCALE}}}};
        kf_character_body_t step_body = {{-KF_FIXED_SCALE, 0, 0}, {kf_fixed_from_int( 100 ), 0, 0}, KF_TRUE};
        const kf_character_config_t step_config = {
            F(0.25), KF_FIXED_SCALE, F(0.5),
            kf_fixed_from_int( 20 ), F(0.01), F(0.001), kf_fixed_from_int( 10 )};
        kf_character_result_t step_result;
        kf_character_step( &step_body, &step_config, world, 2, &step_result );
        CHECK( step_result.stepped == KF_TRUE );
        CHECK( fixed_near( step_body.position.x, 0 ) );
        CHECK( fixed_near( step_body.position.y, F(0.25) ) );
        CHECK( step_body.grounded == KF_TRUE );
    }

    if( failures != 0 )
    {
        return 1;
    }
    puts( "kinefix C tests passed" );
    return 0;
}
